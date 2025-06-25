import moviepy.editor as mp

def convert_mp4_to_gif(input_path, output_path, fps=10, resize_factor=0.5, fuzz=1):
    """
    Converts an MP4 video file to a GIF.

    Args:
        input_path (str): The full path to the input MP4 file.
        output_path (str): The full path where the output GIF will be saved.
        fps (int, optional): The number of frames per second for the GIF. 
                             Defaults to 10. A lower value reduces file size.
        resize_factor (float, optional): A factor to resize the GIF. 
                                         Defaults to 0.5 (half the original size).
        fuzz (int, optional): A fuzz factor for color quantization (0-100). 
                              Higher values increase compression but can reduce quality.
                              Defaults to 1.
    """
    try:
        # Load the video clip
        clip = mp.VideoFileClip(input_path)

        # Resize the clip
        clip_resized = clip.resize(resize_factor)

        # Write the GIF file
        clip_resized.write_gif(output_path, fps=fps, fuzz=fuzz)

        print(f"Successfully converted {input_path} to {output_path}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    # --- Instructions for Use ---
    # 1. Make sure you have a video file named 'input.mp4' in the same 
    #    directory as this script, or provide the full path to your video file.
    # 2. Change the 'output.gif' to your desired output file name.
    # 3. You can adjust the fps (frames per second) and resize_factor to control
    #    the quality and size of the resulting GIF.

    input_video = "/home/lcastri/git/PeopleFlow/shared/experiments/screenrecordings/inference/S2-causal.mp4" 
    output_gif = "/home/lcastri/git/PeopleFlow/shared/experiments/screenrecordings/S2.gif"

    # Example of how to call the function
    convert_mp4_to_gif(input_video, output_gif, fps=1, resize_factor=0.5, fuzz=2)
