import os, cv2
from pathlib import Path
import argparse
import time
import json

from algos import extract_rois, line_segment_detection, get_clusters, cluster_to_contour, extract_polygon
from utils import get_contour_info, log_perf, display_ascii_params

from dev.dynamic_tuners.QIP_tuner import QIPTuner

parser = argparse.ArgumentParser()
parser.add_argument('--tuner', action='store_true', help='specify whether to run the QIP tuner or not')
parser.add_argument('--task', type=str, help='specify the task of program here')
parser.add_argument('--img', type=str, help='specify the path to the input image')

''' 
    Usage (Rough):
        run the program from the workspace ----> </path to workspace>/pct> [IMP]
        python3 main.py --tuner <put false or true> --task <name of task (only use if tuner is false)> --img <path to one image if tuner is enabled>

        'img' arg is only required for tuner app not for QIP_main --> does directory of image processing
    
    
    No Image Pre-Processing for contrast enhancing or image warpping was done the entire raw image was passed as input to the algo 
    so there might be issue for far edge cases involving angle at which image was take or if the image was blurred beyond recongnition,
    - For blurring in the canny edge part images beyond 0.1 sigma are always ignored
'''

# QIP tunning application as now only the high level params are exposed  --> canny edge and houghline params for tuning can be done later if required to solve for edge cases
def run_tuner_app(args=parser.parse_args()):
    image_fp = args.img

    config = None
    with open("config/qip_config.json", 'r') as fp:
        config = json.load(fp)

    if config is None:
        exit(1)

    app = QIPTuner(img_fp=image_fp, config=config)
    app.run_app()


def QIP_main(args=parser.parse_args()):
    task = args.task
    if task is None:
        exit(1)

    # get config data from json file
    config = None
    with open('config/qip_config.json', 'r') as fp:
        config = json.load(fp)

    if config is None:
        exit(1)

    display_ascii_params(config=config)

    image_dir = os.path.join("dataset/samples/")
    image_dir = Path(image_dir)
    extensions = {".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"}
    image_fps = [str(p) for p in image_dir.iterdir() if p.suffix in extensions]

    if not os.path.exists(f"results/{task}/"):
        save_dir = os.makedirs(f"results/{task}", exist_ok=True)
    save_dir = os.path.join(f"results/{task}")


    start = time.perf_counter()
    for i, image_fp in enumerate(image_fps):
        image_bgr = cv2.imread(image_fp)
        ann_img, roi_info = get_contour_info(img_bgr=image_bgr, config=config, contouring_mode=cv2.RETR_EXTERNAL) # maps each ROI to corresponding contours detected in the raw image
        roi = extract_rois(img_bgr=image_bgr, roi_info=roi_info) # extracts the ROI img here
        polygon_info = {} # free structure for now (final output format not fixed yet)


        for j, cnt_id in enumerate(roi):
            roi_img = roi[cnt_id]
            roi_bounds = roi_info[cnt_id]
            x,y,w,h = roi_bounds[0] # the ROI are only cropped not resized so need this info to make sure the final pixels are transformed wrt to original image frames

            _, line_segments = line_segment_detection(img_bgr=roi_img, full=False, annotate=False) # res_img only required if the lines segments are annotated
            clusters = get_clusters(lines=line_segments, config=config) # the unravelling algorithm runs here on each ROIs to extract each polygons as indices of cluster of line-segment
            
            for k, cluster in enumerate(clusters):
                inter_img = roi_img.copy()
                hull, cluster_points = cluster_to_contour(lines=line_segments, cluster=cluster) # the line-segment indices in the cluster is used to extract endpoints corresponding to each segment belonging to the corresponding cluster
                cv2.drawContours(inter_img, [hull], -1, (255, 0, 0), 1) # annotating the convex hull for vis
                inter_img, approx_vertices = extract_polygon(img_bgr=inter_img, config=config, hull=hull, annotate=True) # extarct approx polygon and annotate for vis
                
                # mapping vertices and segment points (unravelled clusters) to cluster_ids for each access outside the algo
                polygon_info[cnt_id] = {
                    "cluster_id": k,
                    "segment_points": cluster_points,
                    "vertices": approx_vertices
                }
                # annotating the vertices of each polygon (due to usage of convex hull for poly approx the vertices are always circularly sorted)
                for l, vertex in enumerate(approx_vertices):
                    vx,vy = vertex
                    vx,vy = x+int(vx),y+int(vy)
                    cv2.circle(ann_img, (vx, vy), 2, (0, 0, 255), -1)
                    cv2.putText(ann_img, f"v-{l}", (vx,vy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)

                # saves
                if not os.path.exists(f"{save_dir}/{task}-{i}/"):
                    sub_dir = os.makedirs(f"{save_dir}/{task}-{i}/", exist_ok=True)
                sub_dir = os.path.join(f"{save_dir}/{task}-{i}/")

                cv2.imwrite(f"{sub_dir}/img-{j}-{k}-{cnt_id}.png", inter_img)
        
        # saves (full annotation)
        cv2.imwrite(f"{save_dir}/{task}-{i}.png", ann_img)    
        end = time.perf_counter()
        print(f"[{task}]-End Time: {end-start}")



if __name__ == '__main__':
    args = parser.parse_args()
    if args.tuner:
        if not args.img:
            parser.error("--img must be provided when --tuner is enabled.")
        args.task = None 
        run_tuner_app(args=args)
    else:

        if not args.task:
            parser.error("--task must be provided when --tuner is disabled.")
        args.img = None
        QIP_main(args=args)
