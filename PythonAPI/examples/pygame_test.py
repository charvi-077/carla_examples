import pygame
import sys

pygame.init()

# Set up screens
screen1 = pygame.display.set_mode((400, 300))
screen2 = pygame.display.set_mode((400, 300))

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Fill screens with different colors
    screen1.fill((255, 255, 255))  # White
    screen2.fill((0, 0, 0))        # Black

    # Update displays
    pygame.display.flip()

pygame.quit()
sys.exit()
