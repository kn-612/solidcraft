SolidCraft with Plasma Nest - test build for Windows 10 and 11
=============================================================

Running it
----------
1. Unzip the folder anywhere (Desktop is fine).
2. Double-click solidcraft.exe.
   The build isn't code-signed. If Windows SmartScreen says "Windows protected your PC",
   click "More info", then "Run anyway".
3. In SolidCraft, click the Workspace switcher (top-left, under the Design icon) and choose
   Plasma Nest. Choosing Design takes you back to modelling; nothing in Design has changed.

Quick test
----------
Setup tab  -> Sample Parts, Import DXF, or drag DXF files from Explorer onto the window.
              Set quantities with the - and + buttons, or click the number and type.
           -> pick a tool in the Tool Library on the right (your Powermax 65 list is built in)
Nest tab   -> Nest Parts. Each DXF gets its own colour. Small parts drop into holes of bigger
              ones ("Part in Part"). Rotation button cycles: none, 180, 90, 45, 15 degrees.
Toolpath   -> parts turn grey; blue = cut path with direction arrows, orange = leads,
              dashed = rapid moves.
              Cut side (side panel): Outside, Inside or No Offset, Reverse Direction, and
              Reverse Cut Side for No Offset. The blue path is the real torch path: it sits half a
              kerf off the drawing (outside the outline and inside holes for Outside, the other
              way round for Inside, on the line for No Offset). Click a part in the list to change only that
              part; with none selected it applies to all parts.
              Leads: lead-in and lead-out each None, Line or Arc, plus overcut. None means the
              torch pierces right on the cut path with no lead.
              Simulate plays the torch along the program (play/pause, speed, drag the timeline).
              Post G-code saves a .ngc file for QtPlasmaC. Copy G-code puts it on the clipboard.
Quote      -> pick a tool, set your $/lb, machine rate, setup fee and markup.

Sheet view
----------
Scroll the mouse wheel to zoom (toward the cursor); drag with the middle button to pan; Fit sheet
frames the whole sheet again. Hover a nested part to highlight it, then left-drag to move it by
hand (Esc while dragging puts it back). A part that ends up closer than the part spacing to
another part, or off the usable sheet, turns red with the parts it touches until you move it clear.

Rotation (Nest tab): none, 180, 90, 45, 15 degrees, or Any (best fit). Best fit searches many
layouts (every 15 degree turn of the first part on each sheet, three packing orders, on all your
PC's cores) for up to 8 seconds, stops early when it reaches the fewest sheets the parts' area
allows, and keeps the layout with the fewest sheets and the least sheet length. Previous / Next
(arrow buttons) step through the sheets.
Small Parts in Holes nests small parts inside the holes of larger ones.

Customer quote (Quote tab)
--------------------------
Prices: $/lb by material, price per pierce, price per cut inch, machine rate, setup fee, scrap
allowance, markup and sales tax. Set the machine rate to 0 to price only by pierces and inches.
Fill in "Customer and job" and "Your business" (company, address, phone, email, website, logo,
terms). Your business details and prices are saved for next time.
Print Quote opens the quote page in your web browser: print it or save it as PDF.
Save Quote saves the same page as an .html file.

Sheet layout
------------
Sheet X is the long side (96" default), Sheet Y the short side (48"). The program origin
(red dot) is the bottom-left corner of the sheet. Parts pack toward the origin and leave the
offcut as one strip at the far X end.

QtPlasmaC notes (please check these before cutting)
---------------------------------------------------
- Programs are laid out like SheetCAM's LinuxCNC PlasmaC post: the same pre-amble (G20 G40 G90,
  M52 P1, M65 P2, M65 P3, M68 E3 Q0, G64 P0.01 Q0.001), material setup, "M3 $0 S1 (plasma
  start)" / "M5 $0 (plasma end)" per cut, the post-amble with "M5 $-1 (backup stop)", the
  statistics and M30. Round holes are single full-circle G3 moves.
- Each program selects its material with  M190 P<tool number>  then  M66 P3 L3 Q2  and uses
  F#<_hal[plasmac.cut-feed-rate]>. So the tool numbers here must match the material numbers in
  your QtPlasmaC material file. Setup -> "QtPlasmaC Materials" writes a material file from this
  tool library (machine_material.cfg; rename it to <your machine>_material.cfg). Back up your
  existing file first.
- Kerf: by default the toolpath itself is offset by half the tool's kerf width, as SheetCAM does,
  so the program has no G41/G42 (the Toolpath panel shows the offset). Tick "Kerf compensation
  in QtPlasmaC" to get the old way back: an unoffset path with G41.1 (or G42.1 when reversed)
  D#<_hal[plasmac_run.kerf-width-f]>, the kerf from the QtPlasmaC material, and a short straight
  entry move that LinuxCNC needs (even with no lead-in). By default outsides cut clockwise and holes
  counter-clockwise, which puts the square edge on the part with a clockwise-swirl torch
  (Hypertherm). Leads are always on the scrap side.
- Parts nested inside another part's hole are cut before that hole.
- Holes smaller than 1.25" run at 60% speed (M67 E3 Q60, back to M67 E3 Q0 after). Both values are on the Toolpath tab.
- Programs are in inches (G20) by default. Untick "Inch program" if your QtPlasmaC config is
  metric.
- Circles and arcs from the DXF are written as true G2/G3 arcs.

Please dry-run the first program (torch off) and cut a test part before trusting sizes.

Known limits in this test build
-------------------------------
- Import DXF takes one file at a time; to add several at once, drag them from Explorer onto the
  Plasma Nest window.
- Plasma Nest settings (parts, nest, prices) aren't saved with the design yet.
- Splines and ellipses in a DXF are cut as short straight segments; lines, arcs and circles are exact.
