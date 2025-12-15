package org.team100.frc2025.CalgamesArm;

import java.awt.Dimension;
import java.util.function.ToDoubleFunction;

import javax.swing.WindowConstants;

import org.jfree.chart.ChartFrame;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberTickUnit;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.VectorRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.VectorSeriesCollection;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.timing.TimedPose;

public class TrajectoryPlotter {
    private final TrajectoryToVectorSeries converter;

    public TrajectoryPlotter(double arrowLength) {
        converter = new TrajectoryToVectorSeries(arrowLength);
    }

    public void plot(Trajectory100 t, String name) {
        VectorSeriesCollection dataSet = new VectorSeriesCollection();
        dataSet.addSeries(converter.convert(t));
        XYPlot plot = new XYPlot(
                dataSet,
                new NumberAxis("X"),
                new NumberAxis("Y"),
                new VectorRenderer());
        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        domain.setRangeWithMargins(xRange(t));
        range.setRangeWithMargins(yRange(t));
        domain.setTickUnit(new NumberTickUnit(1));
        range.setTickUnit(new NumberTickUnit(1));

        ChartFrame frame = new ChartFrame(name, new JFreeChart(plot));
        frame.setPreferredSize(new Dimension(500, 500));
        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        try {
            Thread.sleep(50000);
        } catch (InterruptedException e) {
        }
    }

    Range xRange(Trajectory100 t) {
        return range(t, (p) -> p.state().getPose().pose().getX());
    }

    Range yRange(Trajectory100 t) {
        return range(t, (p) -> p.state().getPose().pose().getY());
    }

    Range range(Trajectory100 t, ToDoubleFunction<TimedPose> f) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (TimedPose p : t.getPoints()) {
            double x = f.applyAsDouble(p);
            if (x > max)
                max = x;
            if (x < min)
                min = x;
        }
        return new Range(min, max);
    }
}
