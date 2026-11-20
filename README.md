# GeoFS-Autopilot-Pro

An upgraded version of the default Autopilot panel in GeoFS. It adds a localizer and glideslope with realistic range.

- LOC range: 30nm
- GS range: 15nm and within 8 degrees of the runway axis

The panel is also visible when not turned on, which means you can set everything before engaging Autopilot.

See these images for reference:

<img width="819" height="61" alt="Screenshot 2026-11-20 at 01 22 58" src="https://github.com/user-attachments/assets/fd52d577-eda2-4a2b-98af-f7c24aa0dd1b" />

Autopilot on with the Localizer captured and the Glideslope armed, this is the usual setup if you're further than 15nm out of the airfield.



<img width="819" height="61" alt="Screenshot 2026-11-20 at 01 19 36" src="https://github.com/user-attachments/assets/75fcfc8b-ad2b-4256-bdaa-9c6b914f4d27" />

Autopilot on with the Localizer available, the Glideslope is still not captured.




<img width="819" height="61" alt="Screenshot 2026-11-20 at 01 19 43" src="https://github.com/user-attachments/assets/23b5caa9-fea2-42b7-9e6a-9a5b6d8e65b6" />

Autopilot on with both the Localizer and Glideslope captured.




<img width="819" height="61" alt="Screenshot 2026-11-20 at 01 20 05" src="https://github.com/user-attachments/assets/43cc5dc8-f86f-4797-ae3b-013c6df7295a" />

Autopilot off but both the Localizer and Glideslope are available.




<img width="819" height="61" alt="Screenshot 2026-11-20 at 01 22 49" src="https://github.com/user-attachments/assets/de4db173-fa27-474b-b3ba-591c431d5e3c" />

Autopilot on with both the Localizer and Glideslope armed, which means they will automatically capture as soon as they're available.



# How do the Localizer and Glideslope work?

LOC:

This turns the aircraft onto the center of the runway axis, but does not influence the altitude, it only makes sure you are alligned with the runway. 
The X-Axis of the runway.

In GeoFS:
- Transparant: Localizer not available or armed, if you click it, it will arm
- Available: Localizer will immediately capture as soon as you click it
- Orange: Localizer armed, it will auto-capture as soon as it available
- Green: Localizer captured

GS:

This glides the aircraft down to the runway, it makes sure you aren't too high, neither too low for final approach. 
The Y-Axis of the runway.

In GeoFS: The button works exactly the same as the Localizer.
