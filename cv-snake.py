import math
import random
import cvzone
from cvzone.HandTrackingModule import HandDetector
import cv2
import numpy as np

width = 1280
height = 720

cap = cv2.VideoCapture(0)
cap.set(3, width)
cap.set(4, height)

detector = HandDetector(detectionCon=0.8, maxHands=1)


class SnakeGame:
    def __init__(self, pathFood):
        self.score = 0
        self.points = []  # all points of the snake
        self.lengths = []  # distance between each points
        self.currentLength = 0  # total length of the snake
        self.allowedLength = 150  # total allowed length
        self.previousHead = 0, 0  # previous head points
        self.imgFood = cv2.imread(pathFood, cv2.IMREAD_UNCHANGED)
        self.hFood, self.wFood, _ = self.imgFood.shape
        self.foodPoints = 0, 0
        self.randomFoodLocation()
        self.gameOver = False

    def randomFoodLocation(self):
        self.foodPoints = random.randint(300, 800), random.randint(200, 600)

    def update(self, imgMain, currentHead):
        if self.gameOver:
            cvzone.putTextRect(imgMain, "GAME OVER",
                               (width // 2, height // 2), 2, 2, (0, 0, 255), 2)
            cvzone.putTextRect(imgMain, f"Score: {self.score}", [
                               width // 2, height // 2 + 50], 2, 2, (0, 0, 255), 2)

        else:
            px, py = self.previousHead
            cx, cy = currentHead

            self.points.append([cx, cy])
            distance = math.hypot(cx-px, cy-py)
            self.lengths.append(distance)
            self.currentLength += distance
            self.previousHead = cx, cy

            if self.currentLength > self.allowedLength:
                for i, length in enumerate(self.lengths):
                    self.currentLength -= length
                    self.lengths.pop(i)
                    self.points.pop(i)
                    if self.currentLength < self.allowedLength:
                        break

            rx, ry = self.foodPoints
            if rx-self.wFood//2 < cx < rx+self.wFood//2 and ry-self.hFood//2 < cy < ry+self.hFood//2:
                self.randomFoodLocation()
                self.allowedLength += 50
                self.score += 1
                print(f"score: {self.score}")

            if self.points:
                for i, points in enumerate(self.points):
                    if i != 0:
                        cv2.line(imgMain, self.points[i-1],
                                 self.points[i], (0, 0, 255), 20)
                cv2.circle(imgMain, self.points[i-1],
                           22, (200, 0, 200), cv2.FILLED)

            rx, ry = self.foodPoints
            imgMain = cvzone.overlayPNG(imgMain, self.imgFood,
                                        (rx-self.wFood//2, ry-self.hFood//2))

            cvzone.putTextRect(imgMain, f"Score: {self.score}",
                               (50, 50), 2, 2, (0, 0, 255), 2)

            # check for collision
            pts = np.array(self.points[:-3], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(imgMain, [pts], False, (0, 200, 0), 4)
            minDist = cv2.pointPolygonTest(pts, (cx, cy), True)
            if -1 <= minDist <= 1:
                print("Hit")
                self.gameOver = True
                self.allowedLength = 150
                self.points = []
                self.lengths = []
                self.currentLength = 0
                self.previousHead = 0, 0
                self.randomFoodLocation()

        return imgMain


game = SnakeGame("donut.png")

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)

    if hands:
        lmList = hands[0]['lmList']
        pointIndex = lmList[8][0:2]
        img = game.update(img, pointIndex)

    cv2.imshow("Image", img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    key2 = cv2.waitKey(2)
    if key2 == ord('r'):
        game.gameOver = False
        game.score = 0
