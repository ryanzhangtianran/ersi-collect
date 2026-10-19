# ERSI-Collect — agent entry points

Single-GPU ERSI-Bench collection packaged as one Docker image (`ersibench-collect`), with LeRobot conversion.
`ersibench/` is a snapshot of ERSI-Bench made by `sync_from_ersibench.sh`: change code in ERSI-Bench, never here.

Skills (`cat` the SKILL.md and follow it):

| Skill | Use it for |
|---|---|
| `skills/create-task/SKILL.md` | write a new task (scene + task json in a dir mounted at /work), check, offline layouts, smoke attempt |
| `skills/collect/SKILL.md` | collect N passed episodes of a task with the image (one container per GPU), LeRobot output, monitoring, stop, verify; rebuild / ship the image |

The image ships one task (pick_biggest_block); every other task is the user's, in their /work dir.
Conventions: one container per GPU and per `<OUT>/<TASK>`; failed episodes are kept in `<OUT>/failed/`, never deleted.
