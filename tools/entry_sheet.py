"""Contact sheet of the newest tour photos, each crop centred on the well rim. usage: tools/entry_sheet.py OUT.jpg WELL..."""
import glob
import os
import sys

import cv2

ims = []
for w in sys.argv[2:]:
    f = sorted(glob.glob(f'evidence/tip/tour_{w}_*.jpg'), key=os.path.getmtime)[-1]
    c = cv2.resize(cv2.imread(f), (584, 292))
    cv2.drawMarker(c, (146, 175), (0, 255, 0), cv2.MARKER_CROSS, 16, 1)
    cv2.putText(c, w, (4, 22), 0, .8, (0, 255, 255), 2)
    ims.append(c)
rows = [cv2.hconcat(ims[i:i + 2]) for i in range(0, len(ims) - 1, 2)]
cv2.imwrite(sys.argv[1], cv2.vconcat(rows))
