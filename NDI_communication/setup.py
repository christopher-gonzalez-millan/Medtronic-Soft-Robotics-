import setuptools

VERSION = '0.0.1'
DESCRIPTION = 'NDI Sensor Communcation'
LONG_DESCRIPTION = 'This package standarizes the communicatioin of Aurora NDI Electromagnetic tracking sensor to the python for using in CU Boulder Medtronic Team 7 2021-2022 senior design'


# Setting up
setuptools.setup(
       # the name must match the folder name 'verysimplemodule'
        name="NDI_Communication",
        version=VERSION,
        author="Christopher Gonzalez-Millan",
        author_email="",
        description=DESCRIPTION,
        long_description=LONG_DESCRIPTION,
        packages=setuptools.find_packages(),
        install_requires=['ndicapi==3.2.8', 'pyserial==3.5'], # add any additional packages that
        # needs to be installed along with your package. Eg: 'caer'

        keywords=['python'],
        classifiers= [
            "Development Status :: 3 - Alpha",
            "Intended Audience :: Education",
            "Programming Language :: Python :: 2",
            "Programming Language :: Python :: 3",
            "Operating System :: MacOS :: MacOS X",
            "Operating System :: Microsoft :: Windows",
        ]
)