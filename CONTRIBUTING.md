# Contributing to Cartesian Motion Base (CMB)

First off, thank you for considering contributing to CMB! It's people like you that make the open-source robotics community such a great place to learn, inspire, and create.

This document provides guidelines for contributing to the repository.

## :lady_beetle: How to Report Bugs
If you find a bug in the source code or a mistake in the documentation, you can help us by submitting an issue to our [GitHub Repository](https://github.com/leledeyuan00/cartesian_motion_base/issues). 
Before submitting a bug report, please check the existing issues to avoid duplicates.

When submitting a bug report, please include:
* Your operating system and ROS 2 distribution (e.g., Ubuntu 22.04, ROS 2 Humble).
* The hardware or simulation environment you are using.
* Steps to reproduce the behavior.
* Relevant terminal outputs or error logs.

## :bulb: How to Request Enhancements
If you have an idea for a new feature or an enhancement to an existing one, please submit an issue outlining your proposal. We welcome discussions on how to make CMB better for contact-rich manipulation tasks!

## :hammer_and_pick:  How to Contribute Code
We use the standard GitHub Pull Request (PR) workflow. 

1. Fork the repository and clone it to your local ROS 2 workspace (`src` directory).
2. Create a new branch for your feature or bug fix:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Make your changes and test them thoroughly. Ensure your code builds successfully using colcon build.
4. Commit your changes with clear and descriptive commit messages.
5. Push to your fork and submit a Pull Request against the main branch of the original repository.
6. In your PR description, explain the changes you made and why they are necessary. If your PR addresses an existing issue, please reference it (e.g., "Fixes #123").

## :scroll: Coding Style
Please follow the ROS 2 C++ coding style guidelines. This includes:
* Using `snake_case` for variable and function names.
* Using `PascalCase` for class names.
* Keeping lines under 80 characters.
* Writing clear and concise comments where necessary.
* Avoiding global variables and using namespaces appropriately.

## :speech_balloon: Need Help?
If you have questions about using the architecture, feel free to open a "Question" issue on GitHub. We will get back to you as soon as possible. 