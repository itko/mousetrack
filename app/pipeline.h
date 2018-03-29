/// \file
/// Maintainer: Felice Serena
///
///

#pragma once


/// Note: This class is mostly mocked, an improved interface and implementation comes later.


namespace MouseTrack {

/// This object is responsible to coordinate the processing steps
///
/// - It knows how to acquire new frames
/// - It knows how to push the frames through the pipeline steps
/// - It informs other objects about (inbetween-)results via an observer pattern (PipelineObserver)
class Pipeline {
public:
    /// Default constructor, create empty pipeline
    Pipeline();

    /// Default destructor
    ~Pipeline();

    /// Start processing
    void start();

    /// Tell the pipeline to terminate as soon as possible.
    /// It will return immediately so you can to other things until the worker finished.
    void stop();

    /// Wait until the pipeline terminates, don't call concurrently
    /// If the worker thread already terminated or never started, it
    /// will return immediately.
    ///
    /// The destructor makes sure, worker thread is terminated properly.
    /// But you probably want to join manually to make sure, all data was delivered.
    ///
    /// Observers are guaranteed not to be called after `join()`returns,
    /// but they might be called while `join()` is executed.
    void join();

};

} // MouseTrack
