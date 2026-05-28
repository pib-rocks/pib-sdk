# Contributing

Welcome to the pib3 contribution guide. We appreciate your interest in improving the library.

## Ways to Contribute

### Report Issues

Found a bug or have a feature request? [Open an issue](https://github.com/mamrehn/pib3/issues) with:

- Clear description of the problem or feature
- Steps to reproduce (for bugs)
- Expected vs actual behavior
- Your environment (OS, Python version, etc.)

### Submit Code

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

See [Development Setup](development.md) for detailed instructions.

### Improve Documentation

Documentation improvements are always welcome:

- Fix typos or unclear explanations
- Add examples
- Improve API documentation
- Add tutorials

### Share Examples

Have an interesting use case? Consider:

- Adding it to the tutorials
- Sharing in discussions
- Creating a blog post

## Quick Start for Contributors

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/pib3.git
cd pib3

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/macOS
# or: venv\Scripts\activate  # Windows

# Install in development mode
pip install -e ".[dev]"

# Run tests
pytest

# Make your changes, then submit PR
```

## Guidelines

### Code Quality

- Follow [PEP 8](https://pep8.org/) style guidelines
- Add docstrings to public functions
- Include type hints
- Write tests for new features

### Commit Messages

Use clear, descriptive commit messages:

```
feat: Add support for custom IK solvers
fix: Handle empty sketch gracefully
docs: Add tutorial for hand poses
test: Add tests for trajectory export
```

### Pull Request Process

1. Update documentation if needed
2. Add tests for new functionality
3. Ensure all tests pass
4. Update CHANGELOG if applicable
5. Request review from maintainers

## Getting Help

- Check existing [issues](https://github.com/mamrehn/pib3/issues)
- Read the [documentation](../index.md)
- Ask questions in [discussions](https://github.com/mamrehn/pib3/discussions)

## Code of Conduct

Be respectful and constructive. We're all here to build something useful together.

## Next Steps

- [Development Setup](development.md) - Set up your development environment
- [Code Style](code-style.md) - Coding conventions and standards
