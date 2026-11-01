SolidCraft with Plasma CAM - test build for Windows 10 and 11
=============================================================

Running it
----------
1. Unzip the folder anywhere (Desktop is fine).
2. Double-click solidcraft.exe.
   The build isn't code-signed. If Windows SmartScreen says "Windows protected your PC",
   click "More info", then "Run anyway".
3. In SolidCraft, click the Workspace switcher (top-left, under the Design icon) and choose
   Plasma CAM. Choosing Design takes you back to modelling; nothing in Design has changed.

Quick test
----------
Setup tab  -> Sample Parts (or Import DXF for your own files, one file at a time)
           -> pick a tool in the Tool Library on the right (your Powermax 65 list is built in)
Nest tab   -> Nest Parts. Each DXF gets its own colour. Small parts drop into holes of bigger
              ones ("Part in Part"). Rotation button cycles: none, 180, 90, 45, 15 degrees.
Toolpath   -> parts turn grey; blue = cut path, orange = lead-in, dashed = rapid moves.
              Post G-code saves a .ngc file for QtPlasmaC. Copy G-code puts it on the clipboard.
Quote      -> pick a tool, set your $/lb, machine rate, setup fee and markup.

Sheet layout
------------
Sheet X is the long side (96" default), Sheet Y the short side (48"). The program origin
(red dot) is the bottom-left corner of the sheet. Parts pack toward the origin and leave the
offcut as one strip at the far X end.

QtPlasmaC notes (please check these before cutting)
---------------------------------------------------
- Each program selects its material with  M190 P<tool number>  then  M66 P3 L3 Q1  and uses
  F#<_hal[plasmac.cut-feed-rate]>. So the tool numbers here must match the material numbers in
  your QtPlasmaC material file. Setup -> "QtPlasmaC Materials" writes a material file from this
  tool library (machine_material.cfg; rename it to <your machine>_material.cfg). Back up your
  existing file first.
- Kerf: G41.1 D#<_hal[plasmac_run.kerf-width-f]>, so the kerf width comes from the QtPlasmaC
  material. Outsides cut clockwise, holes counter-clockwise, lead-ins on the scrap side.
- Holes smaller than 1.25" run at 60% speed (M67 E3 Q60). Both values are on the Toolpath tab.
- Programs are in inches (G20) by default. Untick "Inch program" if your QtPlasmaC config is
  metric.
- Circles and arcs from the DXF are written as true G2/G3 arcs.

Please dry-run the first program (torch off) and cut a test part before trusting sizes.

Known limits in this test build
-------------------------------
- Import DXF takes one file at a time.
- Lead-ins are straight; no lead-outs or arc lead-ins yet.
- Plasma CAM settings (parts, nest, prices) aren't saved with the design yet.
- Splines and ellipses in a DXF are cut as short straight segments; lines, arcs and circles are exact.
