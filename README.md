# MCL_localization

This package implements the particle filter localization using sensor and motion updates from the Pioneer P3-DX robot. To complete this task, you need to make changes in specific files as outlined below.

The main file to work on is located at `src/pf_localisation/pf.py`. Ensure you follow the assignment lab notes for detailed instructions on how to complete the methods. Additional documentation about each method can be found in the source files.

### Important Notes

- **File Changes:**  
  All changes should be made **only in `pf.py`**. This file contains the methods required to complete the localization package.
  - **Inherited Parameters:** If you need to adjust inherited parameters from the `PFLocaliserBase` class, do this directly in the `PFLocaliser` class in `pf.py`.
  
- **Experimentation:**  
  You can experiment with various parameter values in other files, such as `sensor_model.py`, to observe different behaviors and results.

### Building the Package

Follow these steps to build the package:

1. **Move Package:**  
   Move the package to your Colcon workspace, typically the `src` directory.

2. **Rebuild Workspace:**  
   Rebuild the Colcon workspace to integrate any changes.

\`\`\`bash
colcon build --packages-select <package_name>
\`\`\`

> Note: Replace `<package_name>` with the actual name of your package if specified.
"""
