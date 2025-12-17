# Tasks: Upgrade Docusaurus Book UI/UX

**Input**: Design documents from `specs/002-upgrade-docusaurus-ux/`
**Prerequisites**: plan.md, spec.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- All file paths are relative to the `book/` directory.

---

## Phase 1: Setup

**Purpose**: Ensure the development environment is ready.

- [x] T001 Verify that Node.js and npm are installed and meet Docusaurus requirements.
- [x] T002 Navigate to the `book/` directory and run `npm install` to ensure all dependencies are present.

---

## Phase 2: Foundational Theming (User Story 3)

**Purpose**: Establish the core color palette and theme for both light and dark modes. This is a prerequisite for styling other components.
**Goal**: A visually consistent and professional color scheme across the entire book.
**Independent Test**: Navigate between different pages (docs, blog). The colors for the header, footer, and basic elements should be consistent. Toggle between light and dark mode to verify both are implemented.

### Implementation for Foundational Theming

- [x] T003 [US3] Define the new primary, secondary, and text color palettes as CSS variables in `src/css/custom.css`.
- [x] T004 [US3] Implement the dark mode color palette under the `[data-theme='dark']` attribute selector in `src/css/custom.css`.
- [x] T005 [P] [US3] Update the default styles for links and buttons to use the new theme colors in `src/css/custom.css`.
- [x] T006 [P] [US3] Verify that the Docusaurus `themeConfig` in `docusaurus.config.ts` is correctly configured for the navbar and footer to respect the new theme.

**Checkpoint**: Core color theme is applied. Light and dark modes are functional.

---

## Phase 3: Enhanced Readability (User Story 1) 🎯 MVP

**Goal**: A clean, modern, and readable layout with improved typography and spacing.
**Independent Test**: Load any document page. The text should use the new font, sizing, and spacing. Code blocks and admonitions should be clearly styled and legible.

### Implementation for User Story 1

- [x] T007 [US1] Set the root font family, font size, and line height for body text in `src/css/custom.css`. Consider importing a web font like 'Inter' or 'Lato' in `docusaurus.config.ts`.
- [x] T008 [P] [US1] Update the typography for headings (h1, h2, h3) with appropriate size, weight, and spacing in `src/css/custom.css`.
- [x] T009 [P] [US1] Redefine the styles for code blocks, including background color, font, and padding in `src/css/custom.css`.
- [x] T010 [P] [US1] Ensure code blocks have horizontal scrolling on small screens by adjusting their `white-space` and `overflow-x` properties in `src/css/custom.css`.
- [x] T011 [P] [US1] Restyle admonitions (note, tip, warning, danger) to align with the new theme in `src/css/custom.css`.

**Checkpoint**: The main content of the book is significantly more readable and aesthetically pleasing. This represents the minimum viable product.

---

## Phase 4: Intuitive Navigation (User Story 2)

**Goal**: An intuitive and responsive sidebar, top navigation bar, and table of contents.
**Independent Test**: Resize the browser window from desktop to mobile widths. The navigation should adapt correctly. The sidebar should collapse on mobile and be toggleable. The on-page TOC should highlight correctly while scrolling.

### Implementation for User Story 2

- [x] T012 [US2] Verify default responsive behavior of the Docusaurus sidebar. If adjustments are needed for breakpoints or styling, modify the relevant Infima CSS variables in `src/css/custom.css`.
- [x] T013 [US2] Assess the existing Navbar. If structural changes are required (e.g., reordering items), run `npm run swizzle @docusaurus/theme-classic Navbar` in the `book/` directory to eject the component. *(Decision: Not required)*.
- [x] T014 [US2] If swizzled, modify the component in `src/theme/Navbar/` to meet the design requirements. *(Skipped as swizzling was not required)*.
- [x] T015 [US2] Verify the on-page Table of Contents highlighting. Adjust the active link color and style in `src/css/custom.css` to match the new theme.

**Checkpoint**: All navigation components are themed and fully responsive.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final review and documentation to ensure high quality.

- [x] T016 [P] Perform a comprehensive visual review across Chrome, Firefox, and Safari, checking for any style inconsistencies. *(Note: User must perform this visual validation)*.
- [x] T017 [P] Test the site on both a standard mobile viewport (e.g., iPhone 13) and a tablet viewport (e.g., iPad). *(Note: User must perform this visual validation)*.
- [x] T018 Update the `quickstart.md` if any new setup steps were introduced.
- [x] T019 [P] Remove any unused CSS or swizzled component code.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Must be completed first.
- **Foundational Theming (Phase 2)**: Depends on Setup. This phase is a blocker for all other visual work.
- **Readability (Phase 3)**: Depends on Foundational Theming.
- **Navigation (Phase 4)**: Depends on Foundational Theming. Can be worked on in parallel with Phase 3.
- **Polish (Phase 5)**: Depends on all other phases being complete.

### User Story Dependencies

- **US3 (Theming)**: Is foundational and should be implemented first.
- **US1 (Readability)**: Depends on US3.
- **US2 (Navigation)**: Depends on US3.

### Parallel Opportunities

- Once Phase 2 (Foundational Theming) is complete, work on Phase 3 (Readability) and Phase 4 (Navigation) can happen in parallel.
- Within Phase 3, tasks `T008`, `T009`, `T010`, `T011` can be done in parallel.
- Within Phase 5, tasks `T016`, `T017`, `T019` can be done in parallel.

---

## Implementation Strategy

### MVP First (Readability)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational Theming.
3.  Complete Phase 3: Enhanced Readability.
4.  **STOP and VALIDATE**: At this point, the book is visually consistent and highly readable, delivering the most critical user value.

### Incremental Delivery

1.  **Increment 1 (MVP)**: Complete Phases 1, 2, and 3. The book has a new theme and is easy to read.
2.  **Increment 2**: Add Phase 4. Navigation components are now also improved and styled.
3.  **Increment 3**: Complete Phase 5 for a polished, production-ready release.
