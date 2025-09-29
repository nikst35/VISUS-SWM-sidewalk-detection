import cv2
import numpy as np

# ---------- nastavitve ----------
VIDEO = r"C:\Users\nikst\PycharmProjects\OpencvPython\Resourcces\valovi2.mp4"  # ali kamera: 0
SHOW_EDGES = True

# ROI poligon (px) – prilagojen tvoji resoluciji 1280x720
ROI_POLY = np.array([[
    (0, 720), (0, 450), (200, 250), (540, 400),
    (540, 700), (740, 700), (740, 400), (1080, 250),
    (1280, 450), (1280, 720)
]], dtype=np.int32)

# Canny + HoughP parametri
CANNY_LOW  = 65
CANNY_HIGH = 65
HOUGH_RHO = 1
HOUGH_THETA = np.pi / 180
HOUGH_THRESH = 45
HOUGH_MIN_LEN = 40
HOUGH_MAX_GAP = 100

# filtri za “veljavne” črte
MIN_ABS_DY = 200    # črta mora biti dovolj “visoka”
MIN_ABS_DX = 120    # in dovolj “nagnjena”
MIDLINE_X  = 640    # razdelimo sliko na levo/desno polovico

# glajenje presečišč (EMA)
EMA_ALPHA = 0.2

# ---------- pomožne funkcije ----------
def region_of_interest(img, poly):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, poly, 255)
    return cv2.bitwise_and(img, mask)

def line_slope(x1, y1, x2, y2):
    dx = (x2 - x1)
    dy = (y2 - y1)
    if dx == 0:
        return None
    return dy / dx

def intersect_at_y(x1, y1, x2, y2, y_target):
    # vrne x-koordinato, kjer črta (x1,y1)-(x2,y2) seka vodoravnico y=y_target
    dy = y2 - y1
    dx = x2 - x1
    if dy == 0:  # vodoravna – neuporabna
        return None
    m = dy / dx if dx != 0 else None
    if m is None:
        return x1  # navpična črta – x je konstanten
    # y = m(x - x1) + y1  ->  x = (y - y1)/m + x1
    return int((y_target - y1) / m + x1)

def ema(prev, new):
    if prev is None:
        return new
    return int((1 - EMA_ALPHA) * prev + EMA_ALPHA * new)

# ---------- glavna logika ----------
cap = cv2.VideoCapture(VIDEO)
if not cap.isOpened():
    raise RuntimeError("Ne morem odpreti videa/kamere!")

frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  or 1280
frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720

# stanja za glajenje presečišč (zgoraj/spodaj) za L/R
lx_top = lx_bot = None
rx_top = rx_bot = None

frame_num = 0
left_count = right_count = 0
all_lines_count = 0

while True:
    ok, frame = cap.read()
    if not ok:
        break
    frame_num += 1

    # 1) priprava slik
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.bilateralFilter(gray, 9, 200, 200)
    edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH, apertureSize=3)

    # 2) ROI
    edges_roi = region_of_interest(edges, ROI_POLY)

    # 3) HoughP
    lines = cv2.HoughLinesP(edges_roi, HOUGH_RHO, HOUGH_THETA,
                            HOUGH_THRESH, minLineLength=HOUGH_MIN_LEN,
                            maxLineGap=HOUGH_MAX_GAP)

    left_found = False
    right_found = False
    top_y = 300         # zgornja referenčna višina (kot pri tebi)
    bottom_y = 720      # spodnja referenčna višina

    if lines is not None:
        all_lines_count += 1

        # kandidata za levo/desno črto izberemo po naklonu in strani
        best_left  = None  # (x1,y1,x2,y2, |slope|)
        best_right = None

        for ln in lines:
            x1, y1, x2, y2 = ln[0]
            if abs(y2 - y1) < MIN_ABS_DY or abs(x2 - x1) < MIN_ABS_DX:
                continue

            m = line_slope(x1, y1, x2, y2)
            if m is None:
                continue

            # leve črte običajno padajo (negativen naklon) in so na levi polovici
            if m < 0 and x1 < MIDLINE_X and x2 < MIDLINE_X and x1 < x2 and y2 < y1:
                score = abs(m)
                if (best_left is None) or (score > best_left[4]):
                    best_left = (x1, y1, x2, y2, score)

            # desne črte običajno naraščajo (pozitiven naklon) in so na desni polovici
            elif m > 0 and x1 > MIDLINE_X and x2 > MIDLINE_X and x1 < x2 and y1 < y2:
                score = abs(m)
                if (best_right is None) or (score > best_right[4]):
                    best_right = (x1, y1, x2, y2, score)

        # 4) če imamo veljanega levega/desnega kandidata, izračun presečišča in riši
        if best_left is not None:
            x1, y1, x2, y2, _ = best_left
            lx_top_new = intersect_at_y(x1, y1, x2, y2, top_y)
            lx_bot_new = intersect_at_y(x1, y1, x2, y2, bottom_y)
            if lx_top_new is not None and lx_bot_new is not None:
                lx_top = ema(lx_top, lx_top_new)
                lx_bot = ema(lx_bot, lx_bot_new)
                cv2.line(frame, (lx_bot, bottom_y), (lx_top, top_y), (0, 0, 0), 10)
                left_found = True
                left_count += 1

        if best_right is not None:
            x1, y1, x2, y2, _ = best_right
            rx_top_new = intersect_at_y(x1, y1, x2, y2, top_y)
            rx_bot_new = intersect_at_y(x1, y1, x2, y2, bottom_y)
            if rx_top_new is not None and rx_bot_new is not None:
                rx_top = ema(rx_top, rx_top_new)
                rx_bot = ema(rx_bot, rx_bot_new)
                cv2.line(frame, (rx_top, top_y), (rx_bot, bottom_y), (0, 0, 0), 10)
                right_found = True
                right_count += 1

    # 5) napisi status + zapolni poligon, če imava oba roba
    if left_found and right_found and all(v is not None for v in (lx_top, rx_top, rx_bot, lx_bot)):
        cv2.putText(frame, "OBA ROBOVA ZAZNANA", (600, 150), 0, 1.5, (0, 0, 0), 3)
        poly = np.array([[(lx_top, top_y), (rx_top, top_y),
                          (rx_bot, bottom_y), (lx_bot, bottom_y)]], np.int32)
        overlay = frame.copy()
        cv2.fillPoly(overlay, poly, (26, 26, 66))
        frame = cv2.addWeighted(overlay, 0.8, frame, 0.2, 0)
        cv2.line(frame, (lx_top, top_y), (rx_top, top_y), (0, 0, 0), 10)
    elif left_found and not right_found:
        cv2.putText(frame, "LEV ROB JE ZAZNAN", (600, 150), 0, 1.5, (0, 0, 0), 3)
    elif right_found and not left_found:
        cv2.putText(frame, "DESNI ROB JE ZAZNAN", (600, 150), 0, 1.5, (0, 0, 0), 3)
    else:
        cv2.putText(frame, "NI ZAZNANIH ROBOV", (600, 150), 0, 1.5, (0, 0, 0), 5)

    cv2.imshow("image", frame)
    if SHOW_EDGES:
        cv2.imshow("edges", edges)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('x'):
        break

print("stevilo vseh detektiranih setov linij (Hough):", all_lines_count)
print("števec levih:", left_count, " | števec desnih:", right_count)

cap.release()
cv2.destroyAllWindows()
