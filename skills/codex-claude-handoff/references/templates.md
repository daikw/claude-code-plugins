# Templates

Keep templates short and repeatable. Modify field names only if the repo already has a competing standard.

## docs/agent-handoff/README.md

```markdown
# Codex <-> Claude Code Handoff

This folder contains shared artifacts for long-running work.

## Files
- feature-list.json: goals and acceptance checks
- task-queue.json: ordered tasks with owner and status
- progress.md: append-only progress log
- decisions.md: decisions and rationale

## Usage
- Codex: update plan and review gates
- Claude Code: implement tasks and update logs
- Both: keep artifacts current before ending a session
```

## docs/agent-handoff/feature-list.json

```json
{
  "version": 1,
  "features": [
    {
      "id": "F-001",
      "title": "Describe the collaboration protocol",
      "intent": "Codex plans/reviews, Claude Code implements",
      "acceptance": [
        "SKILL.md describes roles and artifacts",
        "Templates for shared files exist"
      ],
      "state": "in-progress"
    }
  ]
}
```

## docs/agent-handoff/task-queue.json

```json
{
  "version": 1,
  "tasks": [
    {
      "id": "T-001",
      "title": "Create shared artifact templates",
      "owner": "codex",
      "state": "todo",
      "depends_on": [],
      "notes": "Keep templates minimal"
    },
    {
      "id": "T-002",
      "title": "Implement templates and update docs",
      "owner": "claude",
      "state": "blocked",
      "depends_on": ["T-001"],
      "notes": "Update progress and decisions log"
    }
  ]
}
```

## docs/agent-handoff/progress.md

```markdown
# Progress Log

- 2026-01-23: Initialized handoff artifacts. Next: define task queue.
- 2026-01-23: Drafted protocol in SKILL.md. Pending review.
```

## docs/agent-handoff/decisions.md

```markdown
# Decisions

- 2026-01-23: Use JSON for feature list and task queue to enable machine checks.
- 2026-01-23: Keep progress and decisions as append-only logs.
```

## Handoff Note (paste into chat)

```markdown
## Handoff
- Status: [what is done]
- Next task: [id + title]
- Files touched: [paths]
- Tests: [commands + results]
- Risks: [regressions or uncertainty]
- Questions: [what needs Codex review]
```
