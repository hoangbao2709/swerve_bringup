# Warehouse 2D Pan Final

- Fixed missing `onContextMenu` prop destructuring that caused the editor source to fail compilation.
- MMB drag pans horizontally and vertically even at 1x zoom.
- Added a small pan overscroll margin to keep navigation usable near viewport bounds.
- Mouse-wheel zoom remains cursor-centered and uses the same pan clamp.
- Disabled browser middle-click auxiliary behavior and set touch-action:none on the SVG canvas.
