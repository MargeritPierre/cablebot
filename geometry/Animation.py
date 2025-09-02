import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation


def animation_test(longueur_1, longueur_2, longueur_3, longueur_4):
    # Longueurs des câbles
    cable_lengths = {
        "Câble 1": longueur_1,
        "Câble 2": longueur_2,
        "Câble 3": longueur_3,
        "Câble 4": longueur_4,
                    }
    
    num_frames = len(cable_lengths["Câble 1"])
    plaque_size = 230  # taille du carré (réduite)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 1000)
    ax.set_zlim(0, 0.1)
    ax.set_title("Déplacement de la plaque dans le plan XY")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    
    # Ajout du repère (flèches des axes)
    ax.quiver(0, 0, 0, 0.2, 0, 0, color='r', arrow_length_ratio=0.1)  # X
    ax.quiver(0, 0, 0, 0, 0.2, 0, color='g', arrow_length_ratio=0.1)  # Y
    ax.quiver(0, 0, 0, 0, 0, 0.2, color='b', arrow_length_ratio=0.1)  # Z
    
    plaque = None
    lines = []
    texts = []
    
    # Coordonnées fixes des points d’attache
    anchor_points = {
        "Câble 1": [1000, 0, 0],
        "Câble 2": [1000, 1000, 0],
        "Câble 3": [0, 0, 0],
        "Câble 4": [0, 1000, 0],
                    }
    
    def update(frame):
        global plaque, lines, texts
        for line in lines:
            line.remove()
        for text in texts:
            text.remove()
        if plaque is not None:
            plaque.remove()
    
        # Longueurs à cet instant
        l1 = cable_lengths["Câble 1"][frame]
        l2 = cable_lengths["Câble 2"][frame]
        l3 = cable_lengths["Câble 3"][frame]
        l4 = cable_lengths["Câble 4"][frame]
    
        # Déplacement du centre de la plaque
        dx = (l2 + l4 - l1 - l3) * 0.25
        dy = (l1 + l2 - l3 - l4) * 0.25
        cx = 0.5 + dx
        cy = 0.5 + dy
        cz = 0  # plan XY
    
        # Coins de la plaque centrée en (cx, cy)
        half = plaque_size / 2
        p1 = [cx - half, cy + half, cz]
        p2 = [cx + half, cy + half, cz]
        p3 = [cx - half, cy - half, cz]
        p4 = [cx + half, cy - half, cz]
    
        # Tracer les câbles et les étiqueter
        lines = []
        texts = []
        for name, anchor, corner in zip(["Câble 1", "Câble 2", "Câble 3", "Câble 4"],
                                        [anchor_points["Câble 1"], anchor_points["Câble 2"], anchor_points["Câble 3"], anchor_points["Câble 4"]],
                                        [p1, p2, p3, p4]):
            # Ligne du câble
            line = ax.plot([anchor[0], corner[0]], [anchor[1], corner[1]], [anchor[2], corner[2]], 'gray')[0]
            lines.append(line)
    
            # Position du texte = milieu du câble
            mid = [(anchor[i] + corner[i]) / 2 for i in range(3)]
            text = ax.text(mid[0], mid[1], mid[2] + 0.01, name, color='black', fontsize=9, ha='center')
            texts.append(text)
    
        # Tracer la plaque (surface carrée)
        X = np.array([[p1[0], p2[0]], [p3[0], p4[0]]])
        Y = np.array([[p1[1], p2[1]], [p3[1], p4[1]]])
        Z = np.array([[cz, cz], [cz, cz]])
        plaque = ax.plot_surface(X, Y, Z, color='skyblue', alpha=0.8)
    
        return lines + [plaque] + texts
    
    ani = animation.FuncAnimation(fig, update, frames=num_frames, interval=600, blit=False)
    plt.show()
