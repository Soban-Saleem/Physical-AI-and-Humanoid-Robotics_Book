#!/bin/bash
# Script Name: generate_design_system.sh
# Description: Generates a basic design system for the textbook platform
# Usage: ./generate_design_system.sh [output_dir]

# Validate input arguments
if [ $# -ne 1 ]; then
    echo "Usage: $0 [output_dir]"
    echo "Example: $0 ./design_system"
    exit 1
fi

OUTPUT_DIR="$1"

# Create the design system directory structure
mkdir -p "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR/components"
mkdir -p "$OUTPUT_DIR/tokens"
mkdir -p "$OUTPUT_DIR/docs"

# Generate design tokens
cat << 'EOF' > "$OUTPUT_DIR/tokens/colors.json"
{
  "colors": {
    "primary": {
      "50": "#eff6ff",
      "100": "#dbeafe",
      "200": "#bfdbfe",
      "300": "#93c5fd",
      "400": "#60a5fa",
      "500": "#3b82f6",
      "600": "#2563eb",
      "700": "#1d4ed8",
      "800": "#1e40af",
      "900": "#1e3a8a"
    },
    "neutral": {
      "50": "#f8fafc",
      "100": "#f1f5f9",
      "200": "#e2e8f0",
      "300": "#cbd5e1",
      "400": "#94a3b8",
      "500": "#64748b",
      "600": "#475569",
      "700": "#334155",
      "800": "#1e293b",
      "900": "#0f172a"
    },
    "success": "#10b981",
    "warning": "#f59e0b",
    "error": "#ef4444"
  }
}
EOF

# Generate typography tokens
cat << 'EOF' > "$OUTPUT_DIR/tokens/typography.json"
{
  "fontFamilies": {
    "primary": "Inter, system-ui, sans-serif",
    "code": "SFMono-Regular, Consolas, monospace"
  },
  "fontSizes": {
    "xs": "0.75rem",
    "sm": "0.875rem",
    "base": "1rem",
    "lg": "1.125rem",
    "xl": "1.25rem",
    "2xl": "1.5rem",
    "3xl": "1.875rem",
    "4xl": "2.25rem",
    "5xl": "3rem",
    "6xl": "3.75rem"
  },
  "lineHeights": {
    "tight": "1.25",
    "snug": "1.375",
    "normal": "1.5",
    "relaxed": "1.625",
    "loose": "2"
  }
}
EOF

# Generate component design
cat << 'EOF' > "$OUTPUT_DIR/components/button.md"
# Button Component

## Purpose
Primary action button for the textbook platform

## Variants
- Primary: For main actions
- Secondary: For secondary actions
- Ghost: For subtle actions

## States
- Default
- Hover
- Active
- Disabled
- Loading

## Accessibility
- Proper ARIA labels
- Keyboard navigation support
- Sufficient color contrast (>4.5:1)
EOF

# Generate accessibility guidelines
cat << 'EOF' > "$OUTPUT_DIR/docs/accessibility.md"
# Accessibility Guidelines

## Standards
All components must meet WCAG 2.1 AA standards

## Color Contrast
- Text on background: minimum 4.5:1 contrast ratio
- Large text: minimum 3:1 contrast ratio

## Keyboard Navigation
- All interactive elements must be keyboard accessible
- Visible focus indicators
- Logical tab order

## Screen Reader Support
- Proper ARIA labels and descriptions
- Semantic HTML structure
- Alternative text for images
EOF

echo "Design system generated successfully in $OUTPUT_DIR"
echo ""
echo "Generated files:"
echo "- tokens/colors.json: Color palette definitions"
echo "- tokens/typography.json: Typography system"
echo "- components/button.md: Component design specifications"
echo "- docs/accessibility.md: Accessibility guidelines"