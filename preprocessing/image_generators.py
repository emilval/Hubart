import os
from google import genai
from google.genai import types
from pathlib import Path

class BaseImageGenerator():

    def __init__(self):
        pass

    def generate_image(self):
        pass

# Need to export GEMINI_API_KEY=""YOUR KEY" to path
class GoogleImageGenerator(BaseImageGenerator):

    def __init__(self):
        self.client = genai.Client()
        self.model = 'gemini-3.1-flash-image'

    def generate_image(self, prompt: str, output_path: Path):

        # Developer API users MUST use generate_content and a Gemini model
        response = self.client.models.generate_content(
            model=self.model,
            contents=prompt,
            config=types.GenerateContentConfig(
                response_modalities=["IMAGE"],
                image_config=types.ImageConfig(
                    aspect_ratio="1:1" # Will generate an image of 1024 x 1024
                )
            )
        )

        # Extract and save the generated image bytes
        for part in response.parts:
            if part.inline_data:
                output_path.write_bytes(part.inline_data.data)
                print(f"Successfully saved: {output_path}")
                return output_path