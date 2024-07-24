import pygame
import carla


def display_latency(vehicle_list, rsu_list, gameDisplay, view_width, font_size=80, text_color = (0, 255, 0), right_margin=600, top_margin=50):
    """
    Prints the latency of the vehicles and RSUs on the pygame display

    Parameters
    ----------
    vehicle_list : list
        List of all vehicles in the simulation
    rsu_list : list
        List of all RSUs in the simulation
    gameDisplay : pygame.display
        The pygame display
    view_width : int
        Width of the display
    font_size : int, optional
        Font size of the text, by default 80
    text_color : tuple, optional
        Color of the text, by default green
    right_margin : int, optional
        Right margin of the text, by default 600
    top_margin : int, optional
        Top margin of the text, by default 50
    """

    font = pygame.font.SysFont(None, font_size)

    #Render text
    text_list = []

    for vehicle_agent in vehicle_list:
        if vehicle_agent.perception.transmission_latency:
            latency = vehicle_agent.perception.transmission_latency_in_sec
            v_text_outline = font.render(f"Vehicle latency is {latency}", True, (0, 0, 0))
            v_text = font.render(f"Vehicle latency is {latency}", True, text_color)
            v_text_position = view_width - right_margin, top_margin + (len(text_list) * (font_size + 10))
            text_list.append((v_text, v_text_outline, v_text_position))

    for rsu in rsu_list:
        if rsu.perception.transmission_latency:
            latency = rsu.perception.transmission_latency_in_sec
            rsu_text_outline = font.render(f"RSU latency is {latency}", True, (0, 0, 0))
            rsu_text = font.render(f"RSU latency is {latency}", True, text_color)
            rsu_text_position = view_width - right_margin, top_margin + (len(text_list) * (font_size + 10))
            text_list.append((rsu_text, rsu_text_outline, rsu_text_position))

    for text, outline, position in text_list:
        x, y = position
        # Draw the outline by slightly offsetting the text in all directions
        gameDisplay.blit(outline, (x - 1, y - 1))
        gameDisplay.blit(outline, (x + 1, y - 1))
        gameDisplay.blit(outline, (x - 1, y + 1))
        gameDisplay.blit(outline, (x + 1, y + 1))
        # Draw the actual text
        gameDisplay.blit(text, position)


def display_rsu(rsu_list, vehicle_list, gameworld):
    """
    Display the RSUs on the pygame display
    
    Parameters
    ----------
    rsu_list : list
        List of all RSUs in the simulation
    vehicle_list : list
        List of all vehicles in the simulation
    gameworld : GameWorld
        The GameWorld object
    """
    rsu_locations = []
    for rsu in rsu_list:
        location = carla.Location(*rsu.perception.global_position[:3])
        rsu_locations.append(location)
        if vehicle_list[0].perception.coop_perception:
            color = carla.Color(0, 255, 0)
        else:
            color = carla.Color(255, 0, 0)
        gameworld.world.debug.draw_point(location, size=0.5, color=color, life_time=-1.0)