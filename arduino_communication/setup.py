import setuptools

VERSION = '0.0.4'
DESCRIPTION = 'Arudion Python Communcation'
LONG_DESCRIPTION = 'This package standarizes the communicatioin of the arduino control board to the python '


# Setting up
setuptools.setup(
       # the name must match the folder name 'verysimplemodule'
        name="arduino_control",
        version=VERSION,
        author="",
        author_email="",
        description=DESCRIPTION,
        long_description=LONG_DESCRIPTION,
        packages=setuptools.find_packages(),
        install_requires=['pyserial==3.5'], # add any additional packages that
        # needs to be installed along with your package. Eg: 'caer'

        keywords=['python', 'first package'],
        classifiers= [
            "Development Status :: 3 - Alpha",
            "Intended Audience :: Education",
            "Programming Language :: Python :: 2",
            "Programming Language :: Python :: 3",
            "Operating System :: MacOS :: MacOS X",
            "Operating System :: Microsoft :: Windows",
        ]
)
