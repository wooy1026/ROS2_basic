># 1. ROS2의 빌드 시스템

## 바이너리 설치위치
- /opt/ros/humble
- `ros run` 또는 `ros launch` 로 해당 패키지 내의 노드를 실행.
설치예제 - `sudo apt install ros-humble-teleop-twist-joy`

## 빌드 시스템/툴 에 대하여
- 빌드 시스템은 단일패키지를 대상으로 하며, 빌드 툴은 전체 패키지를 대상으로 한다.
- ros2 에서는 `colcon` 이라는 빌드 툴을 사용하고, `ament` 라는 빌드 시스쳄을 사용한다.

## 패키지 생성 명령어
- `$ ros2 pkg create [pkg_name] --build-type [build_type] --dependencies [depend_pkg1] [depend_pkg2]`
- build_type 은 c++ 를 사용한다면 ament_cmake를 설정하고
파이썬을 사용한다면 ament_python을 사용한다.
- 단, GUI 프로그램을 작성하다고 한다면 파이썬을 작성하더라도 rqt_plugin 계열을 사용해야 하기에 ament_cmake 를 입력하면 된다.
- 주로 사용하는 의존성패키지는 rclcpp / rclpy / std_msgs

## 패키지 빌드
- `colcon build` 로 전체를 빌드하는데, 이때 빌드 옵션을 추가하여 사용하는게 일반적이다.
- 특정 패키지만 선택하여 빌드하고 할때는 `--pakages-select` 나 `--pakages-up-to` 옵션을 이용하고,
- symlink을 사용하려면 --symlink-install 옵션을 붙여주면 된다. !!주의 심볼릭 옵션을 사용하는것을 추천!!
(symlink는 심볼릭링크 옵션 - doc 참고)

## 패키지 삭제 관리
- `rm -rf src/[pkg1]` 으로 파일을 삭제하고,
/build /install /rog 파일도 같이 삭제한 후, `colcon build`로 빌드 해주면 된다.

## vcstool (버전 컨트롤 시스템 툴)
- wget를 통하여 .repos 라는 파일을 받게 되는데, 이때 이 리포지터리 파일에 vsc타입, 리포지터리주소, 설치해야하는 브랜치 등 다양한 정보가 명시되어 있음.
- `해당파일을 vcs import src < [reponame].repos` 로 패키지를 관리한다.

## etc.
- rosdep : 의존성 관리 툴
- bloom : 바이너리 패키지 관리 툴

<br />

># 2. ROS2의 패키지 파일

## package.xml (패키지 설정파일)
- ros2 - 235 페이지 참고 (서술 예정)

## CMakeLists.txt (빌드 설정 파일)
- ros2 - 236 페이지 참고 (서술 예정)

## setup.py (파이썬 패키지 설정파일)
- ros2 파이썬 패키지에만 사용하는 배포를 위한 설정파일로, c++ 패키지의 package.xml / CMakeLists.txt 파일의 역할을 한다고 생각하면 됨.
- ros2 - 240 페이지 참고 (서술 예정)

## setup.cfg (파이썬 패키지 환경설정 파일)
- setup 파일에서 설정하지 못하는 기타 옵션을 이 파일에 `[develop]` `[install]` 옵션을 사용하여 스크립트의 저장위치를 설정한다.

## plugin.xml (RQt 플러그인 설정 파일)
- RQt 플러그인으로 패키지를 작성(패키지가 rqt에서 불러올 수 있는 플러그인 형태인 경우)할때 필수 구성 요소를 xml 태그로 속성을 기술하는 파일
- [xml 태그속성 doc](https://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin#Attributes_of_library_element_in_plugin.xml)

## etc.
- CHANGELOG.rst : 패키지 변경 로그파일 (패키지의 업데이트 내역을 기술)
- LICENSE : 라이선스 파일 (패키지에 사용된 라이선스 기술)
- README.md : 패키지의 부가설명 파일

<br />

># 3. ROS2 프로그래밍 규칙 (코드)

## 코드 스타일 가이드
- 이름 규칙에는 snake_case, CamelCased, ALL_CAPITALS 와 같이 3종류의 네이밍을 기본으로 사용한다.
- 파일이름 및 변수명, 함수명에는 snake_case 규칙을 따르며, 타입 및 클래스는 CamelCased 규칙을 따른다.
- 단, ROS 인터페이스 파일은 /msg, /srv, /action 폴더에 위치시키며, 인터페이스 파일명은 CamelCased 규칙을 따른다.
- [코드스타일 설명 블로그](https://blog.naver.com/ege1001/220466932974)

## C++ style
- ros2 - 250 페이지 참고 (서술 예정)

## Python Style
1. 기본규칙
    - 파이썬3 (python3.5 이상)을 사용
2. 라인길이
    - 최대 100문자
3. 이름규칙
    - CamelCased / snake_case / ALL_CAPITALS만을 사용한다.
      - CamelCased : 타입 / 클래스
      - snake_case : 파일 / 패키지 / 인터페이스 / 모듈 / 변수 / 함수 / 메소드
      - ALL_CAPITALS : 상수
4. 공백 문자 대 탭 (space vs Tabs)
    - 기본 들여쓰기는 space 4번을 사용한다.
    - 문장 중간의 들여쓰기를 하는 방법과 괄호 및 공백사용 예제 코드
        ```python
        # 들여쓰기 양식
        foo = function_name(var_one, var_two, var_three)

        def long_long_long_long_function_name(
                var_one,
                var_two,
                var_three):
            print(var_three)
        ```
        ```python
        # 괄호 양식 (계산식, 배열 인덱스로 사용. 자료형에 따라 적절하게)
        list = [1, 2, 3, 4]
        dictionary = {'age':30, 'name':'honggildong'}
        tupple = (1, 2, 3, 4)
        ```
5. 주석
    - 문서주식은 """를 사용하며[(PEP 257설명)](https://bigseok.tistory.com/entry/Python-%EB%AC%B8%EC%84%9C%ED%99%94-feat-PEP-257)
    -  구현주석은 #을 이용한다.
6. 기타
    - 모든 문자는 큰따옴표("")가 아닌 작은따옴표('')를 사용하여 표현한다.

<br />

># 4. ROS2프로그래밍(파이썬)

## 초반 세팅
- 개인 패키지 생성 (패키지명 : python_example_pkg)
- 패키지 설정(package.xml / setup.py 수정)
    ### package.xml
    ```xml
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
    <name>python_example_pkg</name>
    <version>0.0.0</version>
    <description>TODO: Package description</description>
    <maintainer email="woo@todo.todo">woo</maintainer>
    <license>TODO: License declaration</license>

    <!--의존성 부분-->
    <depend>rclpy</depend>
    <depend>std_msgs</depend>

    <test_depend>ament_copyright</test_depend>
    <test_depend>ament_flake8</test_depend>
    <test_depend>ament_pep257</test_depend>
    <test_depend>python3-pytest</test_depend>

    <!--build type, C++인 경우 ament_cmake-->
    <export>
        <build_type>ament_python</build_type>
    </export>
    </package>
    ```
    ### setup.py
    - 주의할 점으로 entry_point 옵션의 console_scripts 키를 사용한 실행 파일 설정이다.
    - 아래코드에서 수정한 예를 들어, helloworld_publisher 와 helloworld_subscriber 콘솔 스크리브는 각각 python_example_pkg.helloworld_publisher 모듈과 subscriber 모듈의 main 함수를 호출하게 된다.
    - 해당 설정으로 ros2 run 또는 ros2 launch 명령어로 해당 스크립트를 실행 가능하게 한다.
    ```python
    from setuptools import find_packages, setup

    package_name = 'python_example_pkg'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='woo',
        maintainer_email='woo@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        # entry_points 수정
        entry_points={
            'console_scripts': [
                'helloworld_publisher = python_example_pkg.helloworld_publisher:main',
                'helloworld_subscriber = python_example_pkg.helloworld_subscriber:main'
            ],
        },
    )

    ```
    ### setup.cfg
    - 주의점은 패키지 이름을 기재해야 하며, 나중에 colcon을 이용하여 빌드하게 되면 `/home/[username]/[ws_name]/install/[pkg_name]/lib/[pkg_name]` 와 같은 지정 위치에 실행파일이 생성된다는 점이다.

## 퍼블리셔 노드 작성
- 노드 작성부분은 helloworld_publisher.py와 helloworld_subscriber.py 의 주석을 참고

## 빌드
- `calcon build` 로 빌드를 진행해준다. 패키지를 지정해서 빌드 할 경우 `ROS2의 빌드 시스템` 항목을 참고

<br />

># 5. ROS2 Tips

## 설정스크립트
- 패키지를 빌드 후 설정 스크립트를 일일히 치는것은 귀찮은 일이다.
- `nano ~/.bashrc` 를 이용하여 파일을 열고 최하단에<br />
  `source ~/[workspace_name]/install/local_setup.bash` 를 작성하여 저장하여 사용하자.

## setup.bash VS local_setup.bash

### underlay 와 overlay
- 우리가 ros2같이 바이너리로 설치하여 사용하는(source installation) 워크스페이스를 ROS2 에서는 underlay 라고 한다.
- 개발자가 사용하는 ros2_ws 같은 테스트용 워크스페이스를 ROS2 에서는 overlay라고 한다.
- 즉 overlay 개발환경은 underlay 개발환경에 종속적이게 되며, 설정ㅇ스크립트라고 하는 setup.bash의 호출 순서 및 사용방법이 조금씩 달라지게 된다.

### setup.bash 및 local_setup.bash 사용법
- 두 파일 다 설정스크립트라고 부르며 모든 워크스페이스에 존재한다.
- 다만 local_setup.bash의 경우 이 스크립트가 존재하는 접두사 경로의 모든 패키지에 대한 환경을 설정한다. 즉, 상위 작업공간에 대한 설정은 포함하지 않는다.
- setup.bash의 경우 현재 작업 공간이 빌드될 때 환경에 제공된 다른 모든 작업 공간까지 local_setup.bash 스크립트를 포함하고 있다.
- 워크스페이스가 하나면 문제가 없으나 여러개라면 각 목적에 맞게 사용하여야 한다. 정석은 setup.bash > local_setup.bash 순서로 소싱되는것이다.
- 즉, ~/.bashrc 파일에 `source /opt/ros/humble/setup.bash를 포함하는것이 좋다. (local_setup.bash 이전에 소싱하여야 함을 잊지말자.)

### ROS_DOMAIN_ID
- 도메인 번호를 지정하는 명령어 : `export ROS_DOMAIN_ID=7`

### ROS2 Namespace
- ROS2의 노드가 고유이름을 가지는 것처험 노드에서 사용하는 토픽, 서비스, 액션, 파라미터 간의 네트워크를 이름붙여 그룹화 가능하다.
```
$ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot1
$ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot2
```
- 이때 `--ros-args` 는 일반 커맨드 라인 인자가 아닌 ROS 관련 인자가 온다는것을 의미함 <br />
`-r` 은 이름 재정의를 의미함.

### colcon과 vcstool 명령어 자동완성기능
- ros2 287 페이지 참고 (서술 예정)

<br />

># 6. 토픽, 서비스, 액션 인터페이스
- 이번장에서는 토픽, 서비스, 액션 인터페이스를 신규로 작성하려고 한다.
- 인터페이스를 작성하여 사용하려는 패키지에 포함시켜도 되지만, 경험상 인터페이스로만 구성된 패키지를 별도로 만들어 사용하는것이 관리하기 편하다.
- 패키지 생성 (빌드 타입은 ament_cmake)후 패키지 하위에 action, msg, srv 3개의 폴더를 생성하고 각각 인터페이스 종류에 맞는 아래 파일을 넣는다.
### ArithmeticArgument.msg
```
# Messages
builtin_interfaces/Time stamp
float32 argument_a
float32 argument_b
```

### ArithmeticOperator.srv
```
# Constants
int8 PLUS = 1
int8 MINUS = 2
int8 MULTYPLY = 3
int8 DIVISION = 4

# Request
int8 arithmetic_operator
---
# Response
float32 arithmetic_result
```

### ArithmeticChecker.action
```
# Goal
float32 goal_sum
---
# Result
string[] all_formula
float32 total_sum
---
# Feedback
string[] formula
```

- 인터페이스 비교는 ros2 291 페이지 참고

### package.xml
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>interface_pkg</name>
  <version>0.0.0</version>
  <description>
  ros2_interface_pkg for 2025_mando_ws
  </description>
  <maintainer email="wooy1026@naver.com">woo</maintainer>
  <license>Apache 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>builtin_interfaces</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
- 주요 변화점은 빌드시에 DDS 에서 사용되는 IDL생성과 관련한 rosidl_default_generators 가 사용된다는 점과 실행시에 builtin_interfaces 와 rosidl_default_runtime 가 사용된다는 점이다. 그외에는 일반적인 패키지의 설정이므로 본인에게 맞도록 설정하자. 또한 `<member_of_group>rosidl_interface_packages</member_of_group>` 해당 줄을 추가하여 이 패키지가 다른패키지들이 사용할 메세지, 서비스, 액션 등을 제공한다는 것을 빌드 시스템에 알려주는 역할을 한다.

### CMakeLists.txt
```
cmake_minimum_required(VERSION 3.8)
project(interface_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces) #추가
find_package(rosidl_default_runtime)#추가
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
"msg/ArithmeticArgument.msg"
) #추가

set(srv_files
"srv/ArithmeticOperator.srv"
) #추가

set(action_files
"action/ArithmeticChecker.action"
) #추가

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
) #추가

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```
### 빌드하기
- 빌드 후 문제가 없다면 워크스페이스의 install 폴더 안에 우리가 작성한 ros 인터페이스를 사용하기 위한 파일들이 저장되어 있을것이다. (*.h, *.hpp, *.py 등등)

<br />

># ROS2 패키지 설계(파이썬)

## 패키지 설계
- 프로세스를 목적별로 나누어 노드단위의 프로그램을 작성하고, 노드와 노드간의 데이터 통신을 고려하여 설계해야 한다.

## 노드작성
