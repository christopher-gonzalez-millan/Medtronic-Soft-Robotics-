import setuptools

#TODO: formalize this package during polishing face
setuptools.setup(name='NDISensor',
version='0.1',
description='Class definition for NDI Aurora EM Sensor for object oriented programing',
url='#',
author='Christopher Gonzalez-Millan',
install_requires=['ndicapi==3.2.8', 'pyserial==3.5'],
author_email='',
packages=setuptools.find_packages(),
zip_safe=False)
