from setuptools import setup, find_packages

setup(
    name='obstacle_simulation',
    version='1.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=['numpy', 'scipy', 'matplotlib', 'typing'],
    description='Some custom environment for the gymnasium library',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=2.7.17',
)
