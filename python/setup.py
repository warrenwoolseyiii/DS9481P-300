from setuptools import setup, find_packages

setup(
    name="pyds9481p",
    version="0.1.0",
    author="Roo",
    author_email="roo@example.com",
    description="A Python wrapper for the DS9481P-300 USB to 1-Wire/I2C adapter.",
    long_description=open('../README.md').read(),
    long_description_content_type="text/markdown",
    url="https://github.com/your_username/your_repo", # Replace with your repo URL
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: MacOS",
        "Operating System :: POSIX",
    ],
    python_requires='>=3.6',
)