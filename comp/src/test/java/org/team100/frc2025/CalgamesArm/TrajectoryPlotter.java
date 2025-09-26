package org.team100.frc2025.CalgamesArm;

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
    private final TrajectoryToVectorSeries converter;

    public TrajectoryPlotter(double arrowLength) {
        converter = new TrajectoryToVectorSeries(arrowLength);
    }

    public void plot(Trajectory100 t, String name) {
        VectorSeriesCollection dataSet = new VectorSeriesCollection();
        dataSet.addSeries(converter.convertRotated(t));
        XYPlot plot = new XYPlot(
                dataSet,
                new NumberAxis("X"),
                new NumberAxis("Y"),
                new VectorRenderer());
        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        // make the x and y scales the same
        domain.setRange(-1, 1);
        range.setRange(0, 2);
        domain.setTickUnit(new NumberTickUnit(1));
        range.setTickUnit(new NumberTickUnit(1));

        ChartFrame frame = new ChartFrame(name, new JFreeChart(plot));
        frame.setPreferredSize(new Dimension(500, 500));
        frame.pack();
        frame.setVisible(true);
        try {
            Thread.sleep(50000);
        } catch (InterruptedException e) {
        }
    }
}
