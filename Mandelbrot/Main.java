import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;

public class Main {
    public static void main(String[] args) throws InterruptedException, ExecutionException {
        Instant start = Instant.now();

        int THREADS = 1;
        int WIDTH = 800;
        int HEIGHT = 600;
        ExecutorService executorService = Executors.newFixedThreadPool(THREADS);

        List<Future<List<List<Integer>>>> computedPixels = new ArrayList<>();

        for (int i = 0; i < THREADS; i++) {
            computedPixels.add(executorService.submit(
                    new Mandelbrot(0, i * (HEIGHT / THREADS), WIDTH, i * (HEIGHT / THREADS) + (HEIGHT / THREADS))));

        }

        executorService.shutdown();
        executorService.awaitTermination(10, TimeUnit.SECONDS);

        System.out.println("Took " + start.until(Instant.now(), ChronoUnit.MILLIS) + "ms");

        new MandelbrotImage(computedPixels, WIDTH, HEIGHT).setVisible(true);

    }
}