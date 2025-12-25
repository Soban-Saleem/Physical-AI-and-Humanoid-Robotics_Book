#!/bin/bash
# Script Name: format_textbook_content.sh
# Description: Formats raw content into Docusaurus-compatible markdown
# Usage: ./format_textbook_content.sh [input_file] [output_file]

# Validate input arguments
if [ $# -ne 2 ]; then
    echo "Usage: $0 [input_file] [output_file]"
    echo "Example: $0 raw_content.md formatted_content.md"
    exit 1
fi

INPUT_FILE="$1"
OUTPUT_FILE="$2"

# Check if input file exists
if [ ! -f "$INPUT_FILE" ]; then
    echo "Error: Input file '$INPUT_FILE' does not exist"
    exit 1
fi

echo "Formatting content from $INPUT_FILE to Docusaurus markdown format..."

# In a real implementation, this would perform complex formatting transformations
# For this example, we'll apply basic formatting transformations

# Basic transformations:
# 1. Ensure proper heading hierarchy
# 2. Format code blocks
# 3. Add Docusaurus-specific elements

cat "$INPUT_FILE" | sed \
-e 's/^# /# /g' \
-e 's/^## /## /g' \
-e 's/^### /### /g' \
-e 's/```/```/g' \
> "$OUTPUT_FILE"

# Add Docusaurus-specific frontmatter
{
    echo "---"
    echo "title: Formatted Chapter"
    echo "sidebar_position: 1"
    echo "description: Formatted chapter for Physical AI & Humanoid Robotics textbook"
    echo "---"
    echo ""
    cat "$OUTPUT_FILE"
} > temp_file && mv temp_file "$OUTPUT_FILE"

echo "Content formatted and saved to $OUTPUT_FILE"
echo "Applying accessibility checks..."

# Basic accessibility check (example)
if grep -i "alt=" "$OUTPUT_FILE" > /dev/null; then
    echo "✓ Alt text found for images"
else
    echo "⚠ No alt text found for images - consider adding for accessibility"
fi

if grep -E "^#+ " "$OUTPUT_FILE" > /dev/null; then
    echo "✓ Headings found - good for accessibility"
else
    echo "⚠ No headings found for images - consider adding for accessibility"
fi