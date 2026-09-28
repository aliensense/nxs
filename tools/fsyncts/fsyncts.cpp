// fsyncts: two capture ids in ONE Argus session; per frame, each head's
// start-of-frame timestamp from the capture metadata (TSC, nanoseconds) and
// the difference. Usage: fsyncts <idA> <idB> <modeIdx> <frames> [exposure_us] [fps]
#include <Argus/Argus.h>
#include <Argus/Ext/SensorTimestampTsc.h>
#include <EGLStream/EGLStream.h>
#include <EGL/egl.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace Argus;
using namespace EGLStream;

static void die(const char *m) { fprintf(stderr, "fsyncts: %s\n", m); exit(1); }

int main(int argc, char **argv) {
    int idA = argc > 1 ? atoi(argv[1]) : 2;
    int idB = argc > 2 ? atoi(argv[2]) : 1;
    int modeIdx = argc > 3 ? atoi(argv[3]) : 6;
    int frames = argc > 4 ? atoi(argv[4]) : 200;
    double expUs = argc > 5 ? atof(argv[5]) : 0;
    double fps = argc > 6 ? atof(argv[6]) : 0;

    UniqueObj<CameraProvider> provider(CameraProvider::create());
    ICameraProvider *iProv = interface_cast<ICameraProvider>(provider);
    if (!iProv) die("no camera provider");
    std::vector<CameraDevice *> devs;
    iProv->getCameraDevices(&devs);
    if ((int)devs.size() <= idA || (int)devs.size() <= idB) die("capture id out of range");
    std::vector<CameraDevice *> lr;
    lr.push_back(devs[idA]);
    lr.push_back(devs[idB]);

    UniqueObj<CaptureSession> session(iProv->createCaptureSession(lr));
    ICaptureSession *iSession = interface_cast<ICaptureSession>(session);
    if (!iSession) die("no capture session over the two devices");

    EGLDisplay disp = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (disp == EGL_NO_DISPLAY || !eglInitialize(disp, 0, 0)) die("no EGL display");

    UniqueObj<OutputStreamSettings> settings(iSession->createOutputStreamSettings(STREAM_TYPE_EGL));
    IOutputStreamSettings *iSet = interface_cast<IOutputStreamSettings>(settings);
    IEGLOutputStreamSettings *iEgl = interface_cast<IEGLOutputStreamSettings>(settings);
    if (!iSet || !iEgl) die("no stream settings");
    iEgl->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEgl->setResolution(Size2D<uint32_t>(1920, 1080));
    iEgl->setEGLDisplay(disp);
    iEgl->setMetadataEnable(true);
    iEgl->setMode(EGL_STREAM_MODE_FIFO);
    iEgl->setFifoLength(4);
    iSet->setCameraDevice(lr[0]);
    UniqueObj<OutputStream> sA(iSession->createOutputStream(settings.get()));
    iSet->setCameraDevice(lr[1]);
    UniqueObj<OutputStream> sB(iSession->createOutputStream(settings.get()));
    if (!sA || !sB) die("no output streams");
    UniqueObj<FrameConsumer> cA(FrameConsumer::create(sA.get()));
    UniqueObj<FrameConsumer> cB(FrameConsumer::create(sB.get()));
    IFrameConsumer *iA = interface_cast<IFrameConsumer>(cA);
    IFrameConsumer *iB = interface_cast<IFrameConsumer>(cB);
    if (!iA || !iB) die("no frame consumers");

    UniqueObj<Request> request(iSession->createRequest(CAPTURE_INTENT_PREVIEW));
    IRequest *iReq = interface_cast<IRequest>(request);
    if (!iReq) die("no request");
    iReq->enableOutputStream(sA.get());
    iReq->enableOutputStream(sB.get());
    ICameraProperties *iProps = interface_cast<ICameraProperties>(lr[0]);
    std::vector<SensorMode *> modes;
    iProps->getAllSensorModes(&modes);
    if ((int)modes.size() <= modeIdx) die("sensor mode index out of range");
    ISourceSettings *iSrc = interface_cast<ISourceSettings>(iReq->getSourceSettings());
    if (!iSrc) die("no source settings");
    iSrc->setSensorMode(modes[modeIdx]);
    if (fps > 0) {
        uint64_t ns = (uint64_t)(1e9 / fps);
        iSrc->setFrameDurationRange(Range<uint64_t>(ns, ns));
    }
    if (expUs > 0) {
        uint64_t ns = (uint64_t)(expUs * 1000.0);
        iSrc->setExposureTimeRange(Range<uint64_t>(ns, ns));
    }
    if (iSession->repeat(request.get()) != STATUS_OK) die("repeat failed");
    fprintf(stderr, "fsyncts: session over capture ids %d and %d, mode %d, %d frames\n", idA, idB, modeIdx, frames);

    printf("n\tnumA\tnumB\tA.tsc\tA.tsc2\tB.tsc\tB.tsc2\tA.eof\tB.eof\n");
    for (int n = 0; n < frames; ++n) {
        UniqueObj<Frame> fA(iA->acquireFrame());
        UniqueObj<Frame> fB(iB->acquireFrame());
        if (!fA || !fB) { fprintf(stderr, "fsyncts: stream ended at frame %d\n", n); break; }
        IFrame *ifA = interface_cast<IFrame>(fA);
        IFrame *ifB = interface_cast<IFrame>(fB);
        IArgusCaptureMetadata *aA = interface_cast<IArgusCaptureMetadata>(fA);
        IArgusCaptureMetadata *aB = interface_cast<IArgusCaptureMetadata>(fB);
        if (!aA || !aB) die("no capture metadata on a frame");
        CaptureMetadata *mA = aA->getMetadata();
        CaptureMetadata *mB = aB->getMetadata();
        ICaptureMetadata *iMA = interface_cast<ICaptureMetadata>(mA);
        ICaptureMetadata *iMB = interface_cast<ICaptureMetadata>(mB);
        Ext::ISensorTimestampTsc *tA = interface_cast<Ext::ISensorTimestampTsc>(mA);
        Ext::ISensorTimestampTsc *tB = interface_cast<Ext::ISensorTimestampTsc>(mB);
        if (!ifA || !ifB || !iMA || !iMB || !tA || !tB) die("no timestamp metadata on a frame");
        printf("%d\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\n", n,
               (unsigned long long)ifA->getNumber(), (unsigned long long)ifB->getNumber(),
               (unsigned long long)tA->getSensorSofTimestampTsc(), (unsigned long long)tA->getSensorSofTimestampTsc2(),
               (unsigned long long)tB->getSensorSofTimestampTsc(), (unsigned long long)tB->getSensorSofTimestampTsc2(),
               (unsigned long long)tA->getSensorEofTimestampTsc(), (unsigned long long)tB->getSensorEofTimestampTsc());
    }
    iSession->stopRepeat();
    iSession->waitForIdle();
    return 0;
}
