# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A finding: the sentence a person reads, with its parts apart for a program.
The text is the place it names and the fact on one line, then one `  - `
line per lawful alternative; `where`, `fact` and `alternatives` carry the
same three things as data, beside the text itself."""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Tuple


class Finding(str):
    """One finding. It is its own text, so whatever joins, prints or matches
    findings reads the sentence; the JSON surfaces carry `to_dict()`."""

    _where: str
    _fact: str
    _alternatives: Tuple[str, ...]

    def __new__(cls, where: str, fact: str, alternatives: Iterable[str] = ()) -> "Finding":
        alternatives = tuple(str(a) for a in alternatives)
        head = f"{where}: {fact}" if where else str(fact)
        self = super().__new__(cls, "\n".join([head, *(f"  - {a}" for a in alternatives)]))
        self._where = str(where)
        self._fact = str(fact)
        self._alternatives = alternatives
        return self

    @classmethod
    def of(cls, where: str, problem: Any) -> "Finding":
        """A finding from what a law raised: a refusal that carries its
        alternatives keeps them apart, any other problem is its sentence."""
        fact = getattr(problem, "reason", None) or getattr(problem, "fact", None)
        if fact is not None:
            return cls(where, fact, getattr(problem, "alternatives", ()))
        return cls.parse(where, str(problem))

    @classmethod
    def parse(cls, where: str, text: str) -> "Finding":
        """A finding from a refusal's text: its first line is the fact, its
        `  - ` lines the alternatives."""
        fact, alternatives = parse_refusal(text)
        return cls(where, fact, alternatives)

    @property
    def where(self) -> str:
        return self._where

    @property
    def fact(self) -> str:
        return self._fact

    @property
    def alternatives(self) -> Tuple[str, ...]:
        return self._alternatives

    def to_dict(self) -> Dict[str, Any]:
        return {"where": self._where, "fact": self._fact,
                "alternatives": list(self._alternatives), "text": str(self)}


def parse_refusal(text: str) -> Tuple[str, List[str]]:
    """(fact, alternatives) of a refusal's text."""
    lines = str(text).split("\n")
    alternatives = [line[4:] for line in lines[1:] if line.startswith("  - ")]
    fact = "\n".join([lines[0], *(line for line in lines[1:] if not line.startswith("  - "))])
    return fact, alternatives


def as_data(findings: Iterable[Any]) -> List[Dict[str, Any]]:
    """Findings as the JSON surfaces carry them; a bare sentence is a
    finding that names no place."""
    return [f.to_dict() if isinstance(f, Finding) else Finding.parse("", str(f)).to_dict()
            for f in findings]
