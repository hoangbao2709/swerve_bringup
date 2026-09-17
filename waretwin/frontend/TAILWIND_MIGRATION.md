# Tailwind CSS compatibility update

## What changed

- Tailwind v4 remains the primary utility system via `@import "tailwindcss";`.
- Legacy WareTwin CSS is now isolated inside `@layer components`.
- Global reset is reduced to `@layer base` so Tailwind Preflight remains authoritative.
- Removed `margin: 0; padding: 0;` from the universal selector.
- Removed global `body { overflow: hidden; }`; dashboard overflow is now scoped to `.shell`.
- Kept existing named classes and visual behavior so current screens are not broken.
- No new global element selectors were added to the legacy component layer.

## Recommended rule for new code

Prefer Tailwind utilities in TSX:

```tsx
<div className="flex items-center gap-3 rounded-xl border border-white/10 bg-slate-900 p-4">
```

Only add CSS to `styles.css` when the behavior is truly shared or difficult to express with utilities (3D canvas behavior, complex animations, design tokens, legacy components).

## Tailwind + legacy precedence

`@layer base` -> global defaults
`@layer components` -> existing WareTwin named classes
Tailwind utilities -> per-element overrides

This allows classes such as `flex`, `p-4`, `bg-slate-900`, `text-white`, `overflow-auto`, etc. to override legacy component defaults without adding `!important`.
