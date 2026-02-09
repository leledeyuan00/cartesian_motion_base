import os
import sys
import textwrap

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Cartesian Motion Base Library'
copyright = '2026, Dayuan'
author = 'Dayuan'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'sphinx_copybutton',
    'sphinx_tabs.tabs',
    'sphinx.ext.intersphinx',
    'myst_parser',
    'sphinx_rtd_theme',
    'breathe',
    'exhale',
]

# Breathe Configuration
breathe_projects = {"CartesianMotionBase": "./_doxygen/xml"}
breathe_default_project = "CartesianMotionBase"

# Exhale Configuration
exhale_args = {
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "C++ API Reference",
    "doxygenStripFromPath": "..",
    "createTreeView": True,
    "exhaleExecutesDoxygen": True,
    "exhaleDoxygenStdin": textwrap.dedent('''
        INPUT = ../cartesian_motion_base/include/cartesian_motion_base
        RECURSIVE = YES
        EXTRACT_ALL = YES
        example_path = ../cartesian_motion_test/include/cartesian_motion_test
    '''),
}


source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
html_theme_options = {
    "collapse_navigation": True,
    "sticky_navigation": True,
    "navigation_depth": -1,
    # Only display the logo image, do not display the project name at the top of the sidebar
    # "logo_only": True,
}

html_context = {
    "display_github": True,
    "github_user": "leledeyuan00",
    "github_repo": "cartesian_motion_base",
    "github_version": "master/",
    "conf_py_path": "docs/",
    "source_suffix": ".rst",
}

templates_path = [
    "_templates",
]
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_baseurl = 'https://leledeyuan00.github.io/cartesian_motion_base/'