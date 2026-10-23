"""Hierarchical property tree (zero-dependency, thread-safe)."""

import threading
from typing import Any, Callable, Dict, List, Optional


class PropertyNode:
    def __init__(self, name: str, value: Any = None, ptype: str = "none"):
        self.name = name
        self.value = value
        self.ptype = ptype
        self.children: Dict[str, "PropertyNode"] = {}
        self.listeners: List[Callable[["PropertyNode"], None]] = []
        self._lock = threading.RLock()

    def _coerce(self, value, ptype):
        if ptype == "float":
            return float(value)
        if ptype == "int":
            return int(value)
        if ptype == "bool":
            return bool(value)
        if ptype == "string":
            return str(value)
        return value

    def get_child(self, name: str, create: bool = False, ptype: str = "none") -> "PropertyNode":
        with self._lock:
            if name not in self.children:
                if not create:
                    raise KeyError(f"property not found: {name}")
                self.children[name] = PropertyNode(name, None, ptype)
            child = self.children[name]
            if ptype != "none" and child.ptype == "none":
                child.ptype = ptype
            return child

    def get_value(self, path: str = "", default: Any = None) -> Any:
        node = self
        if path:
            for part in path.strip("/").split("/"):
                if not part:
                    continue
                with node._lock:
                    if part not in node.children:
                        return default
                    node = node.children[part]
        with node._lock:
            return node.value if node.value is not None else default

    def set_value(self, path: str, value: Any, ptype: str = None):
        parts = [p for p in path.strip("/").split("/") if p]
        if not parts:
            with self._lock:
                if ptype is not None:
                    self.ptype = ptype
                if self.ptype != "none":
                    value = self._coerce(value, self.ptype)
                self.value = value
                for cb in list(self.listeners):
                    try:
                        cb(self)
                    except Exception:
                        pass
            return
        node = self
        for part in parts[:-1]:
            node = node.get_child(part, create=True)
        leaf_name = parts[-1]
        with node._lock:
            if leaf_name not in node.children:
                node.children[leaf_name] = PropertyNode(leaf_name, None, ptype or "none")
            child = node.children[leaf_name]
            if ptype is not None:
                child.ptype = ptype
            if child.ptype != "none":
                value = child._coerce(value, child.ptype)
            child.value = value
            for cb in list(child.listeners):
                try:
                    cb(child)
                except Exception:
                    pass

    def add_listener(self, path: str, cb: Callable[["PropertyNode"], None], create: bool = True):
        node = self
        parts = [p for p in path.strip("/").split("/") if p]
        for part in parts:
            node = node.get_child(part, create=create)
        with node._lock:
            node.listeners.append(cb)

    def remove_listener(self, path: str, cb):
        node = self
        parts = [p for p in path.strip("/").split("/") if p]
        for part in parts:
            try:
                node = node.get_child(part, create=False)
            except KeyError:
                return
        with node._lock:
            if cb in node.listeners:
                node.listeners.remove(cb)

    def to_dict(self) -> Any:
        with self._lock:
            if self.children:
                out = {}
                for k, v in list(self.children.items()):
                    out[k] = v.to_dict()
                return out
            return self.value

    def __repr__(self):
        return f"PropertyNode({self.name!r}, {self.value!r}, {self.ptype!r})"

    # convenience
    def get(self, path, default=None):
        return self.get_value(path, default)

    def set(self, path, value, ptype=None):
        self.set_value(path, value, ptype)
