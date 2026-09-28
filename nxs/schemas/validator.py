# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
"""A JSON Schema (draft 2020-12) checker for the keywords the shipped schemas use."""

from __future__ import annotations

import re
from typing import Any, Dict, Iterator, List, Tuple

Path = Tuple[Any, ...]

#: Keywords that describe a schema to a reader and check nothing.
_ANNOTATIONS = frozenset({"$schema", "$id", "$defs", "$comment", "title", "description",
                          "default", "examples", "format", "deprecated"})
_CHECKED = frozenset({"$ref", "type", "enum", "const", "pattern", "minLength", "maxLength",
                      "minimum", "maximum", "exclusiveMinimum", "exclusiveMaximum",
                      "properties", "required", "additionalProperties", "propertyNames",
                      "minProperties", "maxProperties", "dependentRequired", "items",
                      "minItems", "maxItems", "allOf", "anyOf", "oneOf", "not",
                      "if", "then", "else"})


class Error:
    """One violation: where in the document and what the schema says."""

    def __init__(self, path: Path, message: str, allowed: Tuple[Any, ...] = ()) -> None:
        self._path = path
        self._message = message
        self._allowed = tuple(allowed)

    @property
    def absolute_path(self) -> Path:
        return self._path

    @property
    def path(self) -> Path:
        return self._path

    @property
    def message(self) -> str:
        return self._message

    @property
    def allowed(self) -> Tuple[Any, ...]:
        """The values the violated keyword admits (an `enum`'s, a `const`'s,
        a bound's); empty for a keyword that names none."""
        return self._allowed


#: Keywords whose value is one schema, a list of schemas, or a map of name to schema.
_ONE = frozenset({"items", "not", "if", "then", "else", "additionalProperties", "propertyNames"})
_MANY = frozenset({"allOf", "anyOf", "oneOf"})
_NAMED = frozenset({"properties", "$defs"})


def check_keywords(schema: Any) -> None:
    """Refuse a schema that uses a keyword this checker does not implement."""
    if not isinstance(schema, dict):
        return
    unknown = set(schema) - _CHECKED - _ANNOTATIONS
    if unknown:
        raise ValueError(f"schema keyword(s) not implemented: {sorted(unknown)}")
    for key, value in schema.items():
        if key in _ONE:
            check_keywords(value)
        elif key in _MANY:
            for sub in value:
                check_keywords(sub)
        elif key in _NAMED:
            for sub in value.values():
                check_keywords(sub)


def _equal(a: Any, b: Any) -> bool:
    """JSON equality: a boolean is never a number, containers compare by element."""
    if isinstance(a, bool) or isinstance(b, bool):
        return isinstance(a, bool) and isinstance(b, bool) and a == b
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_equal(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_equal(x, y) for x, y in zip(a, b))
    if isinstance(a, (dict, list)) or isinstance(b, (dict, list)):
        return False
    return a == b


def _is_type(instance: Any, name: str) -> bool:
    if name == "integer":
        return (isinstance(instance, int) and not isinstance(instance, bool)) or (
            isinstance(instance, float) and instance.is_integer())
    if name == "number":
        return isinstance(instance, (int, float)) and not isinstance(instance, bool)
    return isinstance(instance, {"object": dict, "array": list, "string": str,
                                 "boolean": bool, "null": type(None)}[name])


def _resolve(root: Dict[str, Any], ref: str) -> Any:
    if not ref.startswith("#/"):
        raise ValueError(f"only local $ref is supported: {ref!r}")
    node: Any = root
    for part in ref[2:].split("/"):
        node = node[part.replace("~1", "/").replace("~0", "~")]
    return node


def _valid(root: Dict[str, Any], schema: Any, instance: Any) -> bool:
    return next(_errors(root, schema, instance, ()), None) is None


def _errors(root: Dict[str, Any], schema: Any, instance: Any, path: Path) -> Iterator[Error]:
    """Violations in the schema's own keyword order, as a reader expects them."""
    if schema is True:
        return
    if schema is False:
        yield Error(path, f"False schema does not allow {instance!r}")
        return
    for key in schema:
        if key == "$ref":
            yield from _errors(root, _resolve(root, schema["$ref"]), instance, path)
        elif key == "type":
            names = [schema[key]] if isinstance(schema[key], str) else list(schema[key])
            if not any(_is_type(instance, n) for n in names):
                yield Error(path, f"{instance!r} is not of type "
                                  f"{', '.join(repr(n) for n in names)}")
        elif key == "enum" and not any(_equal(instance, e) for e in schema["enum"]):
            yield Error(path, f"{instance!r} is not one of {schema['enum']!r}",
                        allowed=schema["enum"])
        elif key == "const" and not _equal(instance, schema["const"]):
            yield Error(path, f"{schema['const']!r} was expected", allowed=[schema["const"]])
        elif key in _STRING and isinstance(instance, str):
            yield from _string(key, schema[key], instance, path)
        elif key in _NUMBER and isinstance(instance, (int, float)) and not isinstance(instance, bool):
            yield from _number(key, schema[key], instance, path)
        elif key in _OBJECT and isinstance(instance, dict):
            yield from _object(root, key, schema, instance, path)
        elif key in _ARRAY and isinstance(instance, list):
            yield from _array(root, key, schema[key], instance, path)
        elif key == "allOf":
            for sub in schema[key]:
                yield from _errors(root, sub, instance, path)
        elif key == "anyOf" and not any(_valid(root, s, instance) for s in schema[key]):
            yield Error(path, f"{instance!r} is not valid under any of the given schemas")
        elif key == "oneOf":
            matched = [s for s in schema[key] if _valid(root, s, instance)]
            if not matched:
                yield Error(path, f"{instance!r} is not valid under any of the given schemas")
            elif len(matched) > 1:
                yield Error(path, f"{instance!r} is valid under each of "
                                  f"{', '.join(repr(s) for s in matched[1:] + matched[:1])}")
        elif key == "not" and _valid(root, schema[key], instance):
            yield Error(path, f"{instance!r} should not be valid under {schema[key]!r}")
        elif key == "if":
            branch = "then" if _valid(root, schema[key], instance) else "else"
            if branch in schema:
                yield from _errors(root, schema[branch], instance, path)


_STRING = frozenset({"pattern", "minLength", "maxLength"})
_NUMBER = frozenset({"minimum", "maximum", "exclusiveMinimum", "exclusiveMaximum"})
_OBJECT = frozenset({"properties", "required", "additionalProperties", "propertyNames",
                     "minProperties", "maxProperties", "dependentRequired"})
_ARRAY = frozenset({"items", "minItems", "maxItems"})


def _string(key: str, value: Any, instance: str, path: Path) -> Iterator[Error]:
    if key == "pattern" and not re.search(value, instance):
        yield Error(path, f"{instance!r} does not match {value!r}")
    if key == "minLength" and len(instance) < value:
        yield Error(path, f"{instance!r} should be non-empty" if value == 1
                    else f"{instance!r} is too short")
    if key == "maxLength" and len(instance) > value:
        yield Error(path, f"{instance!r} is too long")


def _number(key: str, value: Any, instance: Any, path: Path) -> Iterator[Error]:
    if key == "minimum" and instance < value:
        yield Error(path, f"{instance!r} is less than the minimum of {value!r}", allowed=[value])
    if key == "maximum" and instance > value:
        yield Error(path, f"{instance!r} is greater than the maximum of {value!r}", allowed=[value])
    if key == "exclusiveMinimum" and instance <= value:
        yield Error(path, f"{instance!r} is less than or equal to the minimum of {value!r}")
    if key == "exclusiveMaximum" and instance >= value:
        yield Error(path, f"{instance!r} is greater than or equal to the maximum of {value!r}")


def _object(root: Dict[str, Any], key: str, schema: Dict[str, Any], instance: Dict[str, Any],
            path: Path) -> Iterator[Error]:
    value = schema[key]
    if key == "properties":
        for name, sub in value.items():
            if name in instance:
                yield from _errors(root, sub, instance[name], path + (name,))
    elif key == "required":
        for name in value:
            if name not in instance:
                yield Error(path, f"{name!r} is a required property")
    elif key == "additionalProperties":
        extra = [k for k in instance if k not in schema.get("properties", {})]
        if value is False and extra:
            names = sorted(extra, key=str)
            yield Error(path, "Additional properties are not allowed ("
                              f"{', '.join(repr(k) for k in names)} "
                              f"{'was' if len(names) == 1 else 'were'} unexpected)")
        elif value is not True and value is not False:
            for name in extra:
                yield from _errors(root, value, instance[name], path + (name,))
    elif key == "propertyNames":
        for name in instance:
            yield from _errors(root, value, name, path)
    elif key == "minProperties" and len(instance) < value:
        yield Error(path, f"{instance!r} should be non-empty" if value == 1
                    else f"{instance!r} does not have enough properties")
    elif key == "maxProperties" and len(instance) > value:
        yield Error(path, f"{instance!r} has too many properties")
    elif key == "dependentRequired":
        for name, needs in value.items():
            if name in instance:
                for each in needs:
                    if each not in instance:
                        yield Error(path, f"{each!r} is a dependency of {name!r}")


def _array(root: Dict[str, Any], key: str, value: Any, instance: List[Any],
           path: Path) -> Iterator[Error]:
    if key == "items":
        for index, item in enumerate(instance):
            yield from _errors(root, value, item, path + (index,))
    elif key == "minItems" and len(instance) < value:
        yield Error(path, f"{instance!r} should be non-empty" if value == 1
                    else f"{instance!r} is too short")
    elif key == "maxItems" and len(instance) > value:
        yield Error(path, f"{instance!r} is too long")


def errors(schema: Dict[str, Any], instance: Any) -> List[Error]:
    """Every violation of ``instance`` against ``schema``, in document order."""
    return list(_errors(schema, schema, instance, ()))
