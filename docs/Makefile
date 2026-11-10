# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?= -W --keep-going
SPHINXBUILD   ?= sphinx-build
PYTHON        ?= python
EN            = build/source/en
ZH            = build/source/zh
BUILDDIR      = build
STAGE_SCRIPT  = _scripts/stage_sources.py

# Sphinx localizes console progress using the process locale. Keep build logs
# in English without changing the configured language of either HTML site.
ifeq ($(OS),Windows_NT)
SPHINXENV      = set LANGUAGE=en&& set LC_ALL=C&& set LANG=C&&
else
SPHINXENV      = LANGUAGE=en LC_ALL=C LANG=C
endif

# Put it first so that "make" without argument is like "make help".
help: stage
	@$(SPHINXENV) $(SPHINXBUILD) -M help "$(EN)" "$(BUILDDIR)/en" $(SPHINXOPTS) $(O)

.PHONY: help Makefile clean stage

stage:
	@$(PYTHON) $(STAGE_SCRIPT) --language en
	@$(PYTHON) $(STAGE_SCRIPT) --language zh

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile stage
	@$(SPHINXENV) $(SPHINXBUILD) -M $@ "$(EN)" "$(BUILDDIR)/en" $(SPHINXOPTS) $(O)
	@$(SPHINXENV) $(SPHINXBUILD) -M $@ "$(ZH)" "$(BUILDDIR)/zh" $(SPHINXOPTS) $(O)

# Enhanced clean target
clean:
	@echo Cleaning generated documentation under docs/$(BUILDDIR)...
	@echo Documentation source files will be preserved.
	@$(PYTHON) $(STAGE_SCRIPT) --clean
	@echo Documentation clean completed.
