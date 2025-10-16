
from __future__ import annotations

import numpy as np
import cv2


from algos import extract_rois, canny_on_roi, line_segment_detection, get_clusters, cluster_to_contour, extract_polygon
from utils import get_contour_info

class QIPTuner:
    def __init__(self, img_fp: str, config: dict):        
        self.window_name = "QIP-Tuner"
        cv2.namedWindow(f"{self.window_name}", cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1600, 1000)

        # load parameters
        # --------------- TUNABLE PARAMS -----------------
        # Feature extractor params (currently not included)
         # - houghlines params
         # - canny edge detection params
        #-------------------------
        # Algorithm parameters
        self.config = config
        self.min_contour_area = config["MIN_CONTOUR_AREA"] # pix^2 
        self.min_solidity = config["MIN_SOLIDITY"] 
        self.connection_tolerance = config["CONNECTION_TOLERANCE"] # in pix distances
        self.epsilon_factor = config["EPSILON_FACTOR"] # perimeter factor
        #-----------------------------------------

        self.param_specs = {
            "min_contour_area": {"dtype": "int", "scale": 1.0, "max": 10000},
            "min_solidity": {"dtype": "float", "scale": 100.0, "max": 1.0},  # 0–1 scaled to 0–100
            "connection_tolerance": {"dtype": "int", "scale": 1.0, "max": 50},
            "epsilon_factor": {'dtype': "float", "scale": 100.0, "max": 2} # 0-inf scaled and clipped to 0-200
        }
        #-----------------------------------------

        print(f"[INFO]: Loading image......")
        if img_fp is None:
            return
        
        self.img_bgr = cv2.imread(f"{img_fp}") # image in BGR format
        self.ann_img = None # global annotated image

        print("[INFO]: Initial parameter values loaded from config:")
        self.display_ascii_params({
            "min_contour_area": self.min_contour_area,
            "min_solidity": self.min_solidity,
            "connection_tolerance": self.connection_tolerance,
            "epsilon_factor": self.epsilon_factor
        })

        # initialize the tuning sliders
        self.create_tunable_params()


    @staticmethod
    def do_nothing(x):
        pass

    def create_slider(self, name, init_val, max_val, dtype="int", scale=1.0):
        """General-purpose slider creator with dtype control."""
        if dtype == "float":
            init_val = int(init_val * scale)
            max_val = int(max_val * scale)
        elif dtype == "bool":
            init_val = int(bool(init_val))
            max_val = 1

        cv2.createTrackbar(name, self.window_name, init_val, max_val, self.do_nothing)

    def create_tunable_params(self):
        for name, spec in self.param_specs.items():
            val = getattr(self, name)
            self.create_slider(
                name=name,
                init_val=val,
                max_val=spec["max"],
                dtype=spec["dtype"],
                scale=spec["scale"]
            )
    
    def update_params_from_sliders(self):
        """Fetch current slider values and print nicely formatted parameter table."""
        updated = False
        new_params = {}
        for name, spec in self.param_specs.items():
            raw_val = cv2.getTrackbarPos(name, self.window_name)
            if spec["dtype"] == "float":
                val = raw_val / spec["scale"]
            elif spec["dtype"] == "bool":
                val = bool(raw_val)
            else:
                val = int(raw_val)

            # check if changed
            if getattr(self, name) != val:
                setattr(self, name, val)
                updated = True

            new_params[name] = val

        if updated:
            self.display_ascii_params(new_params)

    def display_ascii_params(self, params: dict):
        """Pretty print params as an ASCII table."""
        print("=" * 45)
        print(f"{'QIP TUNER PARAMETER STATUS':^45}")
        print("=" * 45)
        print(f"{'Parameter':<25} | {'Value':>15}")
        print("-" * 45)
        for k, v in params.items():
            if isinstance(v, float):
                print(f"{k:<25} | {v:>15.4f}")
            else:
                print(f"{k:<25} | {v:>15}")
        print("=" * 45)
        print(f"\n")
    
    def cv_window_viewer(self):
        pass
        
    def run_app(self):
        # run the tuner app here
        while True:
            self.update_params_from_sliders()

            # -------------------- ALGO HERE ------------------------------------------------------------------
            # extract ROI info = {cnd_id: bbox_array -->xywh format}
            self.ann_img, roi_info = get_contour_info(
                img_bgr=self.img_bgr, 
                config={"MIN_CONTOUR_AREA": self.min_contour_area, "MIN_SOLIDITY": self.min_solidity}, 
                contouring_mode=cv2.RETR_EXTERNAL
            ) # annotes green external contours and blue bounding rectangle
            roi = extract_rois(img_bgr=self.img_bgr, roi_info=roi_info)
            roi_display_list = []
            houghline_display_list = []

            # algo flow (here)
            for cnt_id in roi:
                roi_img = roi[cnt_id]
                x,y,w,h = roi_info[cnt_id][0]
                # edge detection on ROI
                roi_edges, cleaned_roi_edges = canny_on_roi(roi_img=roi_img) # for plotting and visualization purpose
                # line segment detection
                houghline_img, line_segments = line_segment_detection(img_bgr=roi_img, full=False, annotate=True) # # annotes are green line segments internally runs canny_edge detection
                houghline_display_list.append(houghline_img)
                # connectivity Matrix Generation and generating unravelled clusters
                clusters = get_clusters(lines=line_segments, config={"CONNECTION_TOLERANCE": self.connection_tolerance})
                # cluster_info = {}
                for idx, cluster in enumerate(clusters):
                    roi_cpy = roi_img.copy()
                    hull, collection_points = cluster_to_contour(lines=line_segments, cluster=cluster)
                    # cluster_info[f"cluster{idx}"] = collection_points
                    # get approx polygon off the extracted convex hull contour
                    ann_roi_img, approx_vertices = extract_polygon(
                        img_bgr=roi_cpy,
                        config={"EPSILON_FACTOR": self.epsilon_factor}, 
                        hull=hull, 
                        annotate=True)
                    for v_idx, vertex in enumerate(approx_vertices):
                        vx,vy = vertex
                        vx,vy = x+int(vx),y+int(vy)
                        cv2.circle(self.ann_img, (vx, vy), 2, (0, 0, 255), -1) # red points
                        cv2.putText(self.ann_img, f"v-{v_idx}", (vx,vy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1) # black text

                    roi_display_list.append(ann_roi_img)
            # -------------------- ALGO END ------------------------------------------------------------------

            # ------------------------------ VIEWER PART --------------------------------------------------------------
            # Combine ROI previews + global annotation          
            ann_display = cv2.resize(self.ann_img, (800, 800))
            ann_display_with_title = np.full((30 + ann_display.shape[0], ann_display.shape[1], 3), 255, dtype=np.uint8)
            ann_display_with_title[30:,:] = ann_display
            cv2.putText(ann_display_with_title, "Annotated Full Image", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)

            if len(roi_display_list) > 0:
                roi_previews = [cv2.resize(r, (300, 300)) for r in roi_display_list[:6]]  # show up to 6 ROIs
                roi_strip = cv2.vconcat(roi_previews)
                roi_strip_with_title = np.full((30 + roi_strip.shape[0], roi_strip.shape[1], 3), 255, dtype=np.uint8)
                roi_strip_with_title[30:,:] = roi_strip
                cv2.putText(roi_strip_with_title, "ROI (PolyApproxCnt+ApproxVertex)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)
            else:
                roi_strip_with_title = np.full((100, 300, 3), 255, dtype=np.uint8)
                cv2.putText(roi_strip_with_title, "No ROIs", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)

            # ------- viewer arrangement down here ------------------------
            # sizing parameters
            ann_size = (800*2, 800*2)
            roi_size = (300*2, 300*2)
            roi_spacing = 10*2
            houghline_roi_size = (300*3, 300*3)
            houghline_spacing = 10*2
            margin = 20*2  # spacing between sections

            # ========== LEFT COLUMN (annotated + houghlines) ==========

            # prepare annotated image
            ann_display = cv2.resize(self.ann_img, ann_size)
            ann_display_with_title = np.full((40 + ann_display.shape[0], ann_display.shape[1], 3), 255, dtype=np.uint8)
            ann_display_with_title[40:, :] = ann_display
            cv2.putText(ann_display_with_title, "Annotated Full Image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 5)
            # prepare houghline preview stack
            if len(houghline_display_list) > 0:
                hough_previews = [cv2.resize(r, houghline_roi_size) for r in houghline_display_list]
                spaced_hough = []
                for i, r in enumerate(hough_previews):
                    spaced_hough.append(r)
                    if i < len(hough_previews) - 1:
                        spaced_hough.append(np.full((houghline_spacing, houghline_roi_size[0], 3), 255, dtype=np.uint8))
                hough_strip = cv2.vconcat(spaced_hough)

                hough_strip_with_title = np.full((40 + hough_strip.shape[0], hough_strip.shape[1], 3), 255, dtype=np.uint8)
                hough_strip_with_title[40:, :hough_strip.shape[1]] = hough_strip
                cv2.putText(hough_strip_with_title, "ROI(LineSegments+Endpoints)", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 5)
            else:
                hough_strip_with_title = np.full((100, houghline_roi_size[0], 3), 255, dtype=np.uint8)
                cv2.putText(hough_strip_with_title, "No ROIs", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

            # # --------- resize hough strip to match annotated width ---------
            if hough_strip_with_title.shape[1] != ann_display_with_title.shape[1]:
                hough_strip_with_title = cv2.resize(hough_strip_with_title, (ann_display_with_title.shape[1], hough_strip_with_title.shape[0]))
            # # vertically stack annotated + houghlines (with margin)
            left_column = np.vstack([ann_display_with_title, np.full((margin, ann_display_with_title.shape[1], 3), 255, dtype=np.uint8), hough_strip_with_title])

            # # ROI Previews
            if len(roi_display_list) > 0:
                roi_previews = [cv2.resize(r, roi_size) for r in roi_display_list]
                spaced_rois = []
                for i, r in enumerate(roi_previews):
                    spaced_rois.append(r)
                    if i < len(roi_previews) - 1:
                        spaced_rois.append(np.full((roi_spacing, roi_size[0], 3), 255, dtype=np.uint8))
                roi_strip = cv2.vconcat(spaced_rois)

                roi_strip_with_title = np.full((40 + roi_strip.shape[0], roi_strip.shape[1], 3), 255, dtype=np.uint8)
                roi_strip_with_title[40:, :roi_strip.shape[1]] = roi_strip
                cv2.putText(roi_strip_with_title, "ROI (PolyApproxCnt+ApproxVertex)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 5)
            else:
                roi_strip_with_title = np.full((100, roi_size[0], 3), 255, dtype=np.uint8)
                cv2.putText(roi_strip_with_title, "No ROIs", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

            # Alignment
            max_height = max(left_column.shape[0], roi_strip_with_title.shape[0])
            if left_column.shape[0] < max_height:
                pad = np.full((max_height - left_column.shape[0], left_column.shape[1], 3), 255, dtype=np.uint8)
                left_column = np.vstack([left_column, pad])
            if roi_strip_with_title.shape[0] < max_height:
                pad = np.full((max_height - roi_strip_with_title.shape[0], roi_strip_with_title.shape[1], 3), 255, dtype=np.uint8)
                roi_strip_with_title = np.vstack([roi_strip_with_title, pad])

            # combined view
            combined_view = cv2.hconcat([
                left_column,
                np.full((max_height, margin, 3), 255, dtype=np.uint8),
                roi_strip_with_title
            ])
            cv2.imshow(self.window_name, combined_view)
            # ------------------------------ VIEWER END --------------------------------------------------------------
            # Exit on ESC
            if cv2.waitKey(1) & 0xFF == 27:
                break
        
        self.stop_app()

    def cleanup(self):
        try:
            del self.img_bgr
            del self.ann_img
            del self.config
        except AttributeError:
            pass  # safe guard if already cleaned up

    def stop_app(self):
        self.cleanup()
        cv2.waitKey(1)  # small delay to let OpenCV flush GUI
        cv2.destroyAllWindows()
    