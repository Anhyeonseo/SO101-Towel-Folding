"""In-process rollback experiment; replay equality must be checked by the caller.

Preserves array allocations so existing CUDA graphs keep valid pointers. Opaque
native objects are inventoried, not claimed to be serializable checkpoints.
"""
from __future__ import annotations
import types
import numpy as np


class ArrayStateSnapshot:
    def __init__(self, roots, *, byte_limit=3_000_000_000):
        import warp as wp
        import torch
        self.wp, self.torch = wp, torch
        self.arrays, self.attributes, self.containers, self.opaque = [], [], [], []
        self.bytes = 0
        seen = set()
        def visit(obj, path):
            if obj is None or isinstance(obj, (str, int, float, bool, bytes, types.FunctionType, types.MethodType, types.ModuleType)):
                return
            if id(obj) in seen:
                return
            seen.add(id(obj))
            if isinstance(obj, wp.array):
                value = obj.numpy().copy(); kind = 'warp'
            elif isinstance(obj, torch.Tensor):
                value = obj.detach().cpu().clone(); kind = 'torch'
            elif isinstance(obj, np.ndarray):
                value = obj.copy(); kind = 'numpy'
            else:
                value = None
            if value is not None:
                size = value.numel()*value.element_size() if kind == 'torch' else value.nbytes
                self.bytes += size
                if self.bytes > byte_limit:
                    raise RuntimeError('snapshot exceeds host-memory budget')
                self.arrays.append((obj, value, kind, path))
                return
            if isinstance(obj, dict):
                self.containers.append((obj, obj.copy()))
                for key, value in obj.items(): visit(value, f'{path}[{key}]')
            elif isinstance(obj, (list, set)):
                self.containers.append((obj, obj.copy()))
                for i, value in enumerate(obj): visit(value, f'{path}[{i}]')
            elif isinstance(obj, tuple):
                for i, value in enumerate(obj): visit(value, f'{path}[{i}]')
            elif hasattr(obj, '__dict__') and (obj.__class__.__module__.startswith(('newton', 'mujoco', 'isaaclab', 'mjwarp', 'types')) or isinstance(obj, type)):
                for key,value in vars(obj).items():
                    if key.startswith('__') or isinstance(value,(classmethod,staticmethod,property)) or callable(value):
                        continue
                    self.attributes.append((obj,key,value))
                    visit(value, path+'.'+key)
            else:
                self.opaque.append((path, type(obj).__module__+'.'+type(obj).__name__))
        wp.synchronize()
        for key,obj in roots.items(): visit(obj,key)

    def restore(self):
        self.wp.synchronize()
        for owner,key,value in self.attributes:
            if getattr(owner,key,None) is not value:
                setattr(owner,key,value)
        for target,value in self.containers:
            if isinstance(target,dict): target.clear();target.update(value)
            elif isinstance(target,list): target[:]=value
            else: target.clear();target.update(value)
        for target,value,kind,_path in self.arrays:
            if kind == 'warp': target.assign(value)
            elif kind == 'torch': target.copy_(value.to(target.device))
            else: target[...] = value
        self.wp.synchronize()

    def report(self):
        return {'array_count':len(self.arrays),'host_bytes':self.bytes,
                'arrays':[{'path':path,'kind':kind} for _,_,kind,path in self.arrays],
                'opaque_objects':self.opaque,
                'scope':'in-process array/object rollback, requires empirical replay verification'}
