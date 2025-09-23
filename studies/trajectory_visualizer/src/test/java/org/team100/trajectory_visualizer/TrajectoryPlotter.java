package org.team100.trajectory_visualizer;

import java.awt.Dimension;

import org.jfree.chart.ChartFrame;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberTickUnit;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.VectorRenderer;
import org.jfree.data.xy.VectorSeriesCollection;
import org.team100.lib.trajectory.Trajectory100;

public class TrajectoryPlotter {
    public static void plot(Trajectory100 t, String name) {
        VectorSeriesCollection dataSet = new VectorSeriesCollection();
        dataSet.addSeries(TrajectoryToVectorSeries.convert(t));
        XYPlot plot = new XYPlot(
                dataSet,
                new NumberAxis("X"),
                new NumberAxis("Y"),
                new VectorRenderer());
        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        // make the x and y scales the same
        domain.setRange(0, 10);
        range.setRange(0, 10);
        domain.setTickUnit(new NumberTickUnit(1));
        range.setTickUnit(new NumberTickUnit(1));

        ChartFrame frame = new ChartFrame(
                name,
                new JFreeChart(
                        plot));
        frame.setPreferredSize(new Dimension(500, 500));
        frame.pack();
        frame.setVisible(true);
        try {
<<<<<<< Updated upstream
            Thread.sleep(5000);
=======
            Thread.sleep(50000);
>>>>>>> Stashed changes
        } catch (InterruptedException e) {
        }
    }
}
