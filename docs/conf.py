import os

on_rtd = os.environ.get('READTHEDOCS', None) == 'True'

if not on_rtd:  # only import and set the theme if we're building docs locally
    import sphinx_rtd_theme
    html_theme = 'sphinx_rtd_theme'
    html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

html_logo = "logo.gif"

extensions = [
    'breathe',
    'exhale',
    'sphinx.ext.autosectionlabel'
]

project = u'voxblox'
master_doc = 'index'

html_theme_options = {
    'logo_only': True
}

# Setup the breathe extension
breathe_projects = {"voxblox": "./doxyoutput/xml"}
breathe_default_project = "voxblox"

# Setup the exhale extension
exhale_args = {
    "verboseBuild": False,
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "Library API",
    "doxygenStripFromPath": "..",
    "createTreeView": True,
    "exhaleExecutesDoxygen": True,
    "exhaleUseDoxyfile": True
    "pageLevelConfigMeta": ":github_url: https://github.com/ethz-asl/voxblox"
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'
