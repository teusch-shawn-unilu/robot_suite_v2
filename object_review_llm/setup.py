from setuptools import find_packages, setup

package_name = 'object_review_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'python-dotenv', 'pandas', 'langchain_openai'],
    zip_safe=True,
    maintainer='shawn',
    maintainer_email='shawn.teusch.001@student.uni.lu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'object_review_llm = object_review_llm.csv_object_review_node:main',
    ],
},

)
