## Summary

Describe the purpose of this pull request and the behavior it introduces,
changes, or corrects.

## Scope

List the components and files intentionally affected by this change.

- Component:
- Implementation draft or milestone:
- Related issue:

## Requirements and Architectural Decisions

Identify the functional requirements, non-functional requirements, acceptance
criteria, or architectural decisions affected by this change.

- Requirement or decision:
- Expected impact:

## Verification

Mark every command that was executed successfully.

- [ ] `python -m unittest discover -s tests -p "test_*.py"`
- [ ] `python -m ruff check .`
- [ ] `python -m mypy src`
- [ ] `python tools/check_project_structure.py`
- [ ] `python -m pip check`
- [ ] `python -m ai_logistics_robot`
- [ ] `git diff --check`
- [ ] `python -m build` when packaging behavior changes

Provide the relevant test count and any additional verification:

```text
Test count:
Additional checks:
```

## Safety and Architecture Checklist

- [ ] The change preserves deterministic behavior where required.
- [ ] Public inputs are validated.
- [ ] Confirmed state remains immutable where required.
- [ ] Core packages contain no platform-specific imports.
- [ ] Component responsibilities and dependency directions are preserved.
- [ ] Failure and safety behavior is explicitly tested when affected.
- [ ] No credential, token, personal data, or sensitive configuration is added.
- [ ] Documentation is updated when public behavior changes.
- [ ] Generated or AI-assisted content has been reviewed and understood.

## Deferred Work

List behavior intentionally excluded from this pull request.

- None, or:
- Deferred item:

## Reviewer Notes

Describe any decision, limitation, risk, or specific area that deserves careful
review.
