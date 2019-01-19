import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;
import javax.swing.JFrame;

public class Mandelbrot implements Callable<List<List<Integer>>> {

    private final int startHeight;
    private final int startWidth;
    private final int endHeight;
    private final int endWidth;

    Mandelbrot(int startWidth, int startHeight, int endWidth, int endHeight) {
        this.startWidth = startWidth;
        this.startHeight = startHeight;
        this.endWidth = endWidth;
        this.endHeight = endHeight;

    }

    @Override
    public List<List<Integer>> call() {

        List<List<Integer>> rows = new ArrayList<>();

        for (int y = startHeight; y < endHeight; y++) {

            List<Integer> row = new ArrayList<>();
            for (int x = startWidth; x < endWidth; x++) {
                double zy;
                double zx = zy = 0;
                double ZOOM = 150;
                double cX = (x - 400) / ZOOM;
                double cY = (y - 300) / ZOOM;
                int MAX_ITER = 570;
                int iter = MAX_ITER;
                while (zx * zx + zy * zy < 4 && iter > 0) {
                    double tmp = zx * zx - zy * zy + cX;
                    zy = 2.0 * zx * zy + cY;
                    zx = tmp;
                    iter--;
                }
                row.add(iter);
            }
            rows.add(row);
        }
        return rows;
    }
}

class MandelbrotImage extends JFrame {

    private BufferedImage I;

    public MandelbrotImage(List<Future<List<List<Integer>>>> computedPixels, int width, int height)
            throws ExecutionException, InterruptedException {
        super("Mandelbrot Set");
        setBounds(100, 100, width, height);
        setResizable(false);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        I = new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_INT_RGB);

        for (Future<List<List<Integer>>> computedRows : computedPixels) {
            List<List<Integer>> rows = computedRows.get();
            for (List<Integer> row : rows) {
                for (int pixel : row) {
                    I.setRGB(row.indexOf(pixel), computedPixels.indexOf(computedRows) * rows.size() + rows.indexOf(row),
                            pixel | (pixel << 8));
                }
            }
        }
    }

    @Override
    public void paint(Graphics g) {
        g.drawImage(I, 0, 0, this);
    }

}
