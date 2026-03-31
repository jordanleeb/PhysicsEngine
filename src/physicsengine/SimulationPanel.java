package physicsengine;

import physicsengine.systems.RenderSystem;
import javax.swing.*;
import java.awt.*;

public class SimulationPanel extends JPanel {
    private RenderSystem renderSystem;
    private double alpha = 0;

    public SimulationPanel() {
        setBackground(Color.BLACK);
        setPreferredSize(new Dimension(800, 600));
    }

    public void setRenderSystem(RenderSystem renderSystem) {
        this.renderSystem = renderSystem;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                             RenderingHints.VALUE_ANTIALIAS_ON);
        if (renderSystem != null) renderSystem.render(g2d, alpha);
    }
}