from trainer import Trainer
import numpy as np
import math

# Compare predicted locations with position of the object to grasp/push
def compare_preds_object2grab(primitive_positions, object2grab):
    idx_similars = []
    
    # Which primitives are closer to the objective position?
    for primitive in primitive_positions:
        max_value = max(round(primitive[0], 3), round(object2grab[0], 3))
        if math.isclose(round(primitive[0], 3), round(object2grab[0], 3), rel_tol=(0.005/max_value)):
            idx_similars.append(primitive_positions.index(primitive))
            
    # Go through all primitve positions that had tolerance difference less than 0.005, if any...
    if len(idx_similars) != 0:
        temp_diff = 1
        temp_idx = 0
        for idx in idx_similars:
            diff = abs(primitive_positions[idx][0] - object2grab[0])
            if diff < temp_diff:
                temp_diff = diff
                temp_idx = idx
        return primitive_positions[temp_idx], temp_idx
    # Return object2grab if there aren't any predictions closer to the object
    else:
        return object2grab, 0
    
# Compute 3D locations from predictions, compare to the object to grasp/push and return the prediction of the object to grasp/push
def get_prediction(heightmap_resolution, workspace_limits, valid_depth_heightmap, object2grab, predictions):
    best_pix_ind = []
    pred_value = []
    primitive_positions = []
        
    for i in range(len(predictions)):
        best_pix_ind.append(np.unravel_index(np.argmax(predictions[i]), predictions.shape))
        pred_value.append(predictions[i])
        
        # Compute 3D position of pixel
        best_pix_x = best_pix_ind[-1][2]
        best_pix_y = best_pix_ind[-1][1]
        primitive_positions.append([best_pix_x * heightmap_resolution + workspace_limits[0][0], best_pix_y * heightmap_resolution + workspace_limits[1][0], valid_depth_heightmap[best_pix_y][best_pix_x] + workspace_limits[2][0]])

    best_primitive_position, idx_best_primitive_position = compare_preds_object2grab(primitive_positions, object2grab)
    return best_primitive_position, best_pix_ind[idx_best_primitive_position], pred_value[idx_best_primitive_position]

def compute_position(heightmap_resolution, workspace_limits, valid_depth_heightmap, object2grab, primitive_action, trainer, push_pred=None, grasp_pred=None):
    primitive_position = None
    predicted_value = None
    best_pix_ind = None
    
    # Get pixel location and rotation with highest affordance prediction from heuristic algorithms (rotation, y, x)
    if primitive_action == 'push':
        primitive_position, best_pix_ind, predicted_value = get_prediction(heightmap_resolution, workspace_limits, valid_depth_heightmap, object2grab, push_pred)
    elif primitive_action == 'grasp':
        primitive_position, best_pix_ind, predicted_value = get_prediction(heightmap_resolution, workspace_limits, valid_depth_heightmap, object2grab, grasp_pred)

    best_pix_x = primitive_position[2]
    best_pix_y = primitive_position[1]
    
    # Save predicted confidence value
    trainer.predicted_value_log.append([predicted_value])

    # If pushing, adjust start position, and make sure z value is safe and not too low --> not needed for our case
    '''if primitive_action == 'push':
        finger_width = 0.02
        safe_kernel_width = int(np.round((finger_width/2)/heightmap_resolution))
        local_region = valid_depth_heightmap[max(best_pix_y - safe_kernel_width, 0):min(best_pix_y + safe_kernel_width + 1, valid_depth_heightmap.shape[0]), max(best_pix_x - safe_kernel_width, 0):min(best_pix_x + safe_kernel_width + 1, valid_depth_heightmap.shape[1])]
        if local_region.size == 0:
            safe_z_position = workspace_limits[2][0]
        else:
            safe_z_position = np.max(local_region) + workspace_limits[2][0]
        primitive_position[2] = safe_z_position'''

    # Save executed primitive
    if primitive_action == 'push':
        trainer.executed_action_log.append([0, primitive_position[0], primitive_position[1], primitive_position[2]]) # 0 - push
    elif primitive_action == 'grasp':
        trainer.executed_action_log.append([1, primitive_position[0], primitive_position[1], primitive_position[2]]) # 1 - grasp
    
    return best_pix_ind, primitive_position