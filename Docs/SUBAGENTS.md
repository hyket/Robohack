# Subagents and model selection

Handle small, local tasks yourself. When delegation is authorized, delegate independent,
bounded subtasks when parallel work improves speed or quality. Apply these rules to nested
subagents as well.

## Model and reasoning

Use GPT-6.2 Sol (`gpt-6.2-sol`) for subagents. It is the model ceiling for this project.
Set `reasoning_effort="high"` by default, including for builds, tests, log collection,
implementation, and review. Assess each subtask on its own. Raise reasoning to `xhigh`
or `max` for specific complexity, such as an ambiguous defect across subsystems, a race
condition, or a migration with a risk of data loss.

Before launching, briefly state the chosen model, reasoning level, and why it fits the
subtask. Use settings supported by the current tool and report any limitation. Keep the
primary agent's model settings unless the user asks to change them.

## Launch and review

With `spawn_agent`, set `model`, `reasoning_effort`, and `fork_turns="none"` to select
the model and reasoning explicitly. A full fork inherits the parent settings. If the
approved model is unavailable, report the limitation and handle the work directly.

Example launch parameters:

```json
{
  "task_name": "checks",
  "model": "gpt-6.2-sol",
  "reasoning_effort": "high",
  "fork_turns": "none",
  "message": "Run the existing checks for the assigned change. Leave files untouched and report the commands, exit codes, and relevant failures."
}
```

Give each subagent a self-contained brief with the goal, relevant context, allowed files,
constraints, budget, acceptance criteria, and checks. Assign separate files for concurrent
edits and preserve existing changes. The primary agent owns the final delivery and verifies
the diff and check results before reporting completion.
