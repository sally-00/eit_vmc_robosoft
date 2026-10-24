import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from signal_quality import receiver_limits,quality_flags
from contact_signal_debug import signal_summary,SignalDebugPlot,save_signal_snapshot
from review_session import review_records


def capture(values,clipped=None):
    return {'frames':[{'magnitude_Vrms':list(values),'phase_rad':[0.]*len(values),
                       'clipped':clipped or [False]*len(values)} for _ in range(5)]}


def device(tokens):
    return {'gain_dump':[f'1 2 3 4 1 {g}' for g in tokens],
            'configuration_order_txp_txn_rxp_rxn':[[1,2,3,4] for _ in tokens]}


class QualityTests(unittest.TestCase):
    def test_gain_dependent_ceiling_and_peak_usage(self):
        gains,limits=receiver_limits(device(['1','8','/2','256']),4)
        np.testing.assert_allclose(gains,[1,8,.5,256])
        np.testing.assert_allclose(limits,2.047/(np.sqrt(2)*gains))
        sample=capture([limits[0]/2]*4)
        summary=signal_summary(sample,device=device(['1']*4))
        np.testing.assert_allclose(summary['usage_percent'],50)
        sample['frames'][0]['magnitude_Vrms'][0]=limits[0]*.9
        summary=signal_summary(sample,device=device(['1']*4))
        self.assertAlmostEqual(summary['usage_percent'][0],90)
        self.assertIn('near',quality_flags(summary)[0])

    def test_low_clipped_amplitude_is_not_low_usage(self):
        sample=capture([0.,.000001,.02],[False,True,False])
        summary=signal_summary(sample,device=device(['8']*3))
        self.assertTrue(summary['zero_channels'][0])
        self.assertTrue(np.isnan(summary['usage_percent'][1]))
        self.assertIn('CLIPPED',quality_flags(summary)[1])
        self.assertNotIn('low range',quality_flags(summary)[1])
        figure=Figure(figsize=(8,10));FigureCanvasAgg(figure)
        plot=SignalDebugPlot(figure);plot.device=device(['8']*3)
        plot.update(sample,sample,'Diagnostic test',10.)
        self.assertGreater(plot.usage_clipped.get_ydata()[0],100)
        self.assertEqual(plot.clipped.get_ydata()[0],.001)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'snapshot.json';save_signal_snapshot(path,plot)
            saved=json.loads(path.read_text());self.assertIsNone(saved['summary']['usage_percent'][1])
            self.assertEqual(saved['capture'],sample)
        plot.unavailable('Disconnected')
        self.assertIsNone(plot.summary)
        self.assertEqual(len(plot.usage.get_xdata()),0)

    def test_missing_gains_and_inconsistent_channel_order(self):
        self.assertTrue(np.isnan(receiver_limits({},3)[1]).all())
        self.assertTrue(np.isnan(receiver_limits({'demo':True,**device(['8'])},1)[1]).all())
        with self.assertRaises(ValueError):receiver_limits(device(['8']),2)
        bad=device(['8']);bad['configuration_order_txp_txn_rxp_rxn']=[[2,1,3,4]]
        with self.assertRaises(ValueError):receiver_limits(bad,1)
        for value in (0.,float('nan'),-1.):
            with self.assertRaises(ValueError):receiver_limits({},3,value)

    def test_session_report_flags_without_changing_data(self):
        baseline=capture([.01,0.,.01]);touch=capture([.01,0.,.01],[True,False,False])
        record={'point_id':0,'repeat':0,'xy_mm':[0,0],'baseline':baseline,'contact':touch}
        before=json.dumps(record)
        report=review_records([record],{'device':device(['8']*3)})
        self.assertTrue(np.isfinite(signal_summary(touch,baseline)['rms_change_percent']))
        self.assertEqual(report['touches_with_advisories'],1)
        self.assertEqual(report['touches'][0]['zero_channels'],[2])
        self.assertTrue(any('not clearly above' in x for x in report['touches'][0]['issues']))
        self.assertEqual(json.dumps(record),before)
        json.dumps(report,allow_nan=False)

if __name__=='__main__':unittest.main()
