---
name: codex-claude-handoff
description: Design or refine a Codex <-> Claude Code collaboration protocol for long-running autonomous work. Use when you need role separation (planner/reviewer vs implementer), shared artifacts (plan, task board, progress log, decision log), session start/finish checklists, or handoff templates between Codex and Claude Code across multiple sessions.
---

# Codex <-> Claude Code Handoff Protocol

Codex is the planner/reviewer. Claude Code is the implementer. Keep communication durable, low-latency, and resumable.

## Workflow

1. Clarify the scope and ownership
- Identify repository, target branch, and success definition.
- Assign primary responsibility: Codex plans/reviews, Claude Code implements/tests.
- Decide whether work is single-session or multi-session.

2. Establish shared artifacts (create if missing)
- `docs/agent-handoff/README.md`: how to use the protocol in this repo.
- `docs/agent-handoff/feature-list.json`: long-horizon goals and acceptance checks.
- `docs/agent-handoff/task-queue.json`: ordered tasks with owners and states.
- `docs/agent-handoff/progress.md`: running log of work with timestamps.
- `docs/agent-handoff/decisions.md`: decisions, tradeoffs, and rationale.

3. Define the operating contract
- Codex provides a plan, constraints, and review checklist.
- Claude Code implements tasks, updates artifacts, and requests review.
- Both sides keep artifacts current after each meaningful change.

4. Run each session with checklists
- **Start**: read all shared artifacts; confirm the next task.
- **During**: update `progress.md` as work completes; keep tasks in sync.
- **Finish**: write a clean handoff note; list blockers and open questions.

## Artifact Rules

- Prefer small, append-only logs for progress and decisions.
- Use optimistic concurrency for shared JSON (include a `version` field).
- Keep tasks granular enough to finish in 30-120 minutes.
- Tag tasks with owner (`codex` or `claude`) and state.

## Roles

### Codex (planner/reviewer)
- Produce a short plan with milestones and review gates.
- Maintain `feature-list.json` and `task-queue.json` health.
- Review Claude Code outputs for risk, regression, and missing tests.
- Decide when to split work into new tasks.

### Claude Code (implementer)
- Execute tasks in order; update files and tests.
- Keep `progress.md` and `decisions.md` current.
- Record blockers, assumptions, and local commands run.
- Request review with a concise summary and a verification list.

## Templates

Use the templates in `references/templates.md` to create or refresh artifacts.
