#### Back to [README](/README.md)

# Autonomous Routs

During auto, the intake and flywheel run continuously to enable faster collecting and shooting of Notes.
See PathPlanner for the auto routes. We should print those out for use at competition.

Available auto routines:
* None: does nothing. Don't use this!
* DriveForward: drives 2m forward, scoring 2 points for leaving the starting zone. Starting position: 1m from the wall, 3.5m south of the speaker.
* DriveForwardAltPosition: same as DriveForward, but starts further north on the field. Starting position: 1m from the wall, 2.5m south of the speaker.
* ShootNoteAndDrive: Shoots a preloaded note, then drives out of the starting zone. Starting position: directly in front of the speaker.
* ShootPickShoot: Shoots a preloaded note, then drives forward to pick up the note and shoots it too. Starting position: directly in front of the speaker.
* All4: Shoots a preloaded note, then drives to pick up the note by the stage pillar, pulls up in front of the speaker to shoot, drives straight forward to pick up and shoot the note in front of the speaker, then picks up the other close note and shoots it.