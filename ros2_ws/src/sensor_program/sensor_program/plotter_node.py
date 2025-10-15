from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import time
import threading
import os
import datetime
import rclpy

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10
        )
        self.data = []
        self.timestamps = []
        self.plot_dir = '/data'  # Carpeta donde se guardarán los gráficos

        # Crear carpeta si no existe
        os.makedirs(self.plot_dir, exist_ok=True)

        # Hilo aparte para generar el gráfico cada 5 segundos
        self.timer_thread = threading.Thread(target=self.plot_loop, daemon=True)
        self.timer_thread.start()

    def listener_callback(self, msg):
        try:
            value = int(msg.data.split(':')[1].split()[0])
            self.data.append(value)
            self.timestamps.append(time.time())
            self.get_logger().info(f'Recibido dato: {value}')
        except ValueError:
            self.get_logger().warn(f'No se pudo convertir "{msg.data}" a número')

    def plot_loop(self):
        while rclpy.ok():
            time.sleep(5)
            if len(self.data) > 0:
                # Crear nombre de archivo con fecha y hora actual
                timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                plot_filename = f"sensor_plot_{timestamp_str}.png"
                plot_path = os.path.join(self.plot_dir, plot_filename)

                plt.figure()
                plt.plot(self.timestamps, self.data, marker='o')
                plt.xlabel('Tiempo (s)')
                plt.ylabel('Valor del sensor')
                plt.title('Evolución del sensor')
                plt.grid(True)
                plt.savefig(plot_path)
                plt.close()
                self.get_logger().info(f'Nuevo gráfico guardado en {plot_path}')

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
