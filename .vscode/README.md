# Color Coding System for Paint Machine

This project uses VSCode color coding to visually distinguish between code related to each side of the painting process.

## Side Color Coding

The color scheme for each side is:

| Side  | Color             | Hex Code | Description          |
|-------|-------------------|----------|----------------------|
| Back  | Green             | #4CAF50  | Top-right corner     |
| Front | Blue              | #2196F3  | Bottom-left corner   |
| Left  | Amber/Gold        | #FFC107  | Bottom-right corner  |
| Right | Pink/Magenta      | #E91E63  | Top-left corner      |

## How to Use in Code

Add comments with the appropriate tag to colorize code sections:

```cpp
// back: This is a comment for back-related code
// front: This is a comment for front-related code
// left: This is a comment for left-related code
// right: This is a comment for right-related code
```

## Other Special Comments

The project also uses other special comments:

```cpp
//! Important step in a sequence (red)
//? Question or something that needs attention (blue)
//todo: Something that needs to be done (orange)
//* Important information (green)
// Strikethrough text (gray, crossed out)
```

## Implementation Details

The color coding is implemented using the Better Comments extension and custom VS Code settings.

The configuration is stored in `.vscode/settings.json`. 