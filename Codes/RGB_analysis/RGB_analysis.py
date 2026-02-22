import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
import sys

class RoverHistogramApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ITU Rover Team - Çift Bölge Spektral Analiz")
        self.setGeometry(100, 100, 1300, 750)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)

        # Sol taraf: Kamera Görüntüsü
        self.image_label = QLabel()
        self.main_layout.addWidget(self.image_label)

        # Sağ taraf: Histogramlar
        self.hist_layout = QVBoxLayout()
        self.main_layout.addLayout(self.hist_layout)

        self.figure, (self.ax_top, self.ax_bottom) = plt.subplots(2, 1, figsize=(6, 8))
        self.canvas = FigureCanvas(self.figure)
        self.hist_layout.addWidget(self.canvas)

        # Kamera Başlatma
        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def process_region(self, frame_roi, ax, title):
        """Bölgedeki sarı alanları bulur, çizer ve histogramı hesaplar."""
        # 1. Sarı maskeleme ve İnce Kontur tespiti
        hsv = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Konturları bul (İnce çizgi için)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Sadece sarı bölgelerin olduğu maske üzerinden histogram hesapla (Ekran görüntüsündeki gibi)
        colors = ('b', 'g', 'r')
        plot_colors = ('blue', 'green', 'red')
        
        ax.clear()
        for i, col in enumerate(colors):
            # Maske kullanılarak sadece sarı alanların RGB dağılımı alınır
            hist = cv2.calcHist([frame_roi], [i], mask, [256], [0, 256])
            ax.plot(hist, color=plot_colors[i], linewidth=1.5)
            
        ax.set_title(title, fontsize=10)
        ax.set_xlim([0, 256])
        ax.grid(True, alpha=0.2)

        # Orijinal ROI üzerine çok ince kırmızı çizgileri çiz
        cv2.drawContours(frame_roi, contours, -1, (0, 0, 255), 1)

    def update_frame(self):
        ret, frame = self.cap.read()
        if not ret: return

        height, width, _ = frame.shape
        mid_y = height // 2

        # Görüntüyü ikiye böl
        top_roi = frame[0:mid_y, :]
        bottom_roi = frame[mid_y:height, :]

        # İşlemleri gerçekleştir
        self.process_region(top_roi, self.ax_top, "RGB Histogram (Top Region)")
        self.process_region(bottom_roi, self.ax_bottom, "RGB Histogram (Bottom Region)")

        self.canvas.draw()

        # Ayrım çizgisini BEYAZ yap
        cv2.line(frame, (0, mid_y), (width, mid_y), (255, 255, 255), 2)
        
        # PyQt'ye gönder
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_img = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_img).scaled(700, 700, Qt.KeepAspectRatio))

    def closeEvent(self, event):
        self.cap.release()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoverHistogramApp()
    window.show()
    sys.exit(app.exec_())