# Build the Documentation

The documentation is built as independent English and Chinese Sphinx sites.
Run all commands from the `sdk_glue/docs` directory.

## Install dependencies

```bash
python -m pip install --user -r requirements.txt
```

## Generate HTML

On Linux or another environment with GNU Make:

```bash
make html
```

On Windows:

```bat
make.bat html
```

The English and Chinese sites are generated in `build/en/html` and
`build/zh/html`, respectively. Temporary source trees are generated below
`build/source`; building never writes copied content into `source`.
Sphinx console progress is always printed in English for both site builds.

Warnings are treated as errors by default. To check external links separately,
run `make linkcheck` or `make.bat linkcheck` while network access is available.

To remove generated files without touching documentation sources, run
`make clean` or `make.bat clean`.
