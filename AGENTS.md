# Robohack

Robohack is the Participant container for the Unitree Go1 simulator hackathon. The
current implementation covers T1. See the [technical stack](Docs/STACK.md).

- Read the relevant [hackathon requirements](Docs/HACKATHON.md) before changing the
  container, entrypoint, or simulator interfaces.
- Preserve `/entrypoint.sh ping` and its JSON protocol. Keep `supported_tasks` aligned
  with the task modes the container actually implements.
- Use the [build and smoke check in the README](README.md#сборка-и-проверка) for runtime
  changes. The participant must run without network access during the T1 smoke check.
- Keep edits within the assigned task and preserve the user's existing changes.
- Read the [subagent rules](Docs/SUBAGENTS.md) before delegating.
