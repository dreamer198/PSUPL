# Documentation Assets

## Algorithm Framework

Place the `algorithm_framework.png` image file in this directory.

The framework diagram should illustrate the complete pipeline of the unified point-line RANSAC algorithm, showing:

1. Input data types (point clouds、line segments、point clouds + line segments)
2. Sampling strategies (three-point, point-line, two-line)
3. Plane construction and evaluation process
4. Iterative optimization with probability models
5. Boundary optimization and connectivity verification
6. Final segmentation results

## Image Requirements

- **Format**: PNG (preferred) or JPG
- **Resolution**: Minimum 1200px width for clarity
- **File name**: `algorithm_framework.png`
- **Background**: White or transparent for better integration

## Usage in README

The image is referenced in the main README.md file as:
```markdown
![Algorithm Framework](docs/algorithm_framework.png)
```