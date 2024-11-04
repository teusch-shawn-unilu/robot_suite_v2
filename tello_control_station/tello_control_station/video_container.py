import pygame

class VideoContainer():
    def __init__(self, display):
        self.display = display 
        
        self.battery = 50
        self.display_mode = "m"
        self.face_boxes = []
        self.bg_image = None
        self.selected_face_box = None
        self.nose = None
        self.skeleton_layer = None
        
        
    def update_battery(self, battery_level):
        self.battery = battery_level
        
    def update_bg_image(self, image):
        self.bg_image = image
        
    def update_face_boxes(self, boxes):
        self.face_boxes = boxes
            
    def update_selected_face_box(self, box, nose):
        self.selected_face_box = box
        self.nose = nose
        
    def update_skeleton_layer(self, layer):
        self.skeleton_layer = layer
        
    def update_image_frame(self):
        if self.bg_image is not None:
            self.display.blit(self.bg_image, (0, 0))
            
        if self.display_mode == "f":
            for rec in self.face_boxes:
                pygame.draw.rect(self.display, "red", rec, 2)
        
            if self.selected_face_box is not None and self.nose is not None:
                pygame.draw.rect(self.display, "green", self.selected_face_box, 2)
                pygame.draw.circle(self.display, "green", self.nose, 5)
                
        elif self.display_mode == "h" and self.skeleton_layer is not None:
            self.display.blit(self.skeleton_layer, (0, 0))
                
        pygame.draw.rect(self.display, "red", (280, 20, 400, 40))
        pygame.draw.rect(
            self.display, "green", (280, 20, 4 * self.battery, 40)
        )
        pygame.display.flip()
    
    
            
        