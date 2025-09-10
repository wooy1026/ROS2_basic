import rclpy
from rclpy.node import Node #! rclpy의 node 클래스를 사용
from rclpy.qos import QoSProfile #! 퍼블리셔의 QOS 설정을 위함
from std_msgs.msg import String #! 퍼블리셔의 메세지 타입은 string 메세지 인터페이스


class HelloworldPublisher(Node): #! 이 노드의 메인클래스는 HelloworldPublisher 이고 Node 클래스를 이용해 상속해 사용

    def __init__(self): #! 최초 설정
        super().__init__('helloworld_publisher') #todo 클래스 생성자 정의로 부모클래스의 생성자를 호출하고 노드 이름을 helloworld_publisher로 설정?
        qos_profile = QoSProfile(depth=10) #! QOS프로파일을 호출 및 뎁스를 10으로 설정:통신상태가 원활하지 않거나 문제 발생시 퍼블리시 데이터를 버퍼에 10개 저장
        self.helloworld_publisher = self.create_publisher(String, 'helloworld', qos_profile) #! 토픽메세지 타입 설정 (메세지타입,토픽명,QoS)
        self.timer = self.create_timer(1, self.publish_helloworld_msg) #! 콜백 함수 실행 (x초마다, 실행함수)
        self.count = 0 #! 콜백함수에 사용하는 카운트 값

    def publish_helloworld_msg(self): #! 콜백함수 시작
        msg = String() #! 퍼블리시 할 메세지 타입 지정
        msg.data = 'Hello World: {0}'.format(self.count) #! 실제 데이터 저장, Hello World 와 같이 올라가는 카운트가 데이터
        self.helloworld_publisher.publish(msg) #! 데이터 퍼블리시
        self.get_logger().info('Published message: {0}'.format(msg.data)) #! 터미널 창에 로거를 출력, 종류에 따라 info, warning, error 등으로 표시
        self.count += 1 #! 카운트 증가

def main(args=None): #! 메인함수 시작
    rclpy.init(args=args) #! 작성중인 파이썬 스크립트를 ROS2 노드로서 동작시키기 위한 필수 초기화 함수
    node = HelloworldPublisher() #! 위 클래스를 변수로 생성하고
    try:
        rclpy.spin(node) #! spin 시켜서 지정된 콜백함수를 실행시킴
    except KeyboardInterrupt: #! 아래부터는 종료와 같은 인터럽트 상황에서 노드를 소멸시키는 부분
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

