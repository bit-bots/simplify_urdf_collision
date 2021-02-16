from urdf_parser_py.urdf import URDF, Mesh
from color import COLORS

class URDFHandler:
    def __init__(self, filename, excluded_links):
        self.robot = URDF.from_xml_file(filename)
        links = self.robot.link_map
        self.collision_models = {}
        for link_name, link  in links.items():
            if link_name in excluded_links:
                continue
            c_single_link = []
            for c in link.collisions:
                if type(c.geometry) == Mesh:
                    c_single_link.append(c)
            if c_single_link:
                self.collision_models[link_name] = c_single_link

        sum = 0
        for _,c in self.collision_models.items():
            sum += len(c)
        print(f"{COLORS.OKGREEN}{sum} mesh collision models to be simplified were found in the URDF (excluded links excluded){COLORS.ENDC}")  
    

    def get_filenames(self, interactive):
        """
            returns a dict of link_name: [collision_model_1, collision_model_2 ...]
            if interactive is enabled, uses command line input to 
        """
        if interactive:
            while True:
                links = list(self.collision_models.keys())
                for i,l in enumerate(links):
                    print(f"{COLORS.OKBLUE}[{i}]{COLORS.ENDC} {l} ({len(self.collision_models[l])})") 
                selected_l = input(f"Enter {COLORS.OKBLUE}e{COLORS.ENDC} to exit or link number to deselect collision models {COLORS.OKBLUE}[0]...[{len(links)-1}]{COLORS.ENDC}:")
                if selected_l == "e":
                    break
                else:
                    selected_l_int = int(selected_l)
                    while True:
                        # delete link from collision model paths dict if there are no more models
                        if len(self.collision_models[links[selected_l_int]]) == 0:
                            self.collision_models.pop(links[selected_l_int])
                            break

                        # print model paths
                        for j, c in enumerate(self.collision_models[links[selected_l_int]]):
                            print(f"{COLORS.OKBLUE}[{j}]{COLORS.ENDC} {c.geometry.filename}")
                        selected_c = input(f"Enter e to exit or collision model number to deselect {COLORS.OKBLUE}[0]...[{len(self.collision_models[links[selected_l_int]])-1}]{COLORS.ENDC}:")
                        if selected_c == "e":
                            break
                        else:
                            selected_c_int = int(selected_c)
                            # delete selected collision model path from list
                            del self.collision_models[links[selected_l_int]][selected_c_int]
        return self.collision_models

    def write_urdf(self, filename):
        s = self.robot.to_xml_string()
        f = open(filename, "w+")
        f.write(s)
        f.close()

if __name__ == "__main__":
    u = URDFHandler("/home/jasper/ws/src/simplify_urdf_collision/wolfgang_old.urdf")
        