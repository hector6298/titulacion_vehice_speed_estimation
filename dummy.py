
#foreground detection
##just a script for notes (do not run)


## USING TRADITIONAL PROCESSING

forExt = TraditionalForegroundExtractor()
cap = cv2.VideoCapture("/home/hector/speed/speed_DS/v1.mp4")
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True:
    grayFrame = convertImg2Grayscale(frame)
    foregroundFrame = forExt.maskCarsAtFrame(grayFrame)
    edgeMap = getEdgeMap(foregroundFrame,100,200)
    lines = getLinesFromEdges(edgeMap,(-15,15), 10, 0)
    empty = np.zeros((edgeMap.shape[0],edgeMap.shape[1],3))
    empty[:,:,0] = edgeMap
    linesImg = drawLinesOnImg(empty, lines)
    cv2.imshow('Frame',linesImg)
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
  else: 
    break
cap.release()
cv2.destroyAllWindows()

#### USING SEGMENTATION

cap = cv2.VideoCapture("/home/hector/speed/speed_DS/v1.mp4")
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True:
    frame = cv2.resize(frame, (512,512))
    grayFrame = convertImg2Grayscale(frame)
    maskedFrame = forExt.segment(frame, grayFrame)
    edgeMap = getEdgeMap(maskedFrame,100,200)
    lines = getLinesFromEdges(edgeMap,(-15,15), 10, 0)
    empty = np.zeros((edgeMap.shape[0],edgeMap.shape[1],3))
    empty[:,:,0] = edgeMap
    linesImg = drawLinesOnImg(empty, lines)
    cv2.imshow('Frame',linesImg)
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
  else: 
    break
cap.release()
cv2.destroyAllWindows()